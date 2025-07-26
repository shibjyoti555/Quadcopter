#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include <Vcc.h>

const float VccMin = 0.0;
const float VccMax = 4.49;
const float VccCorrection = 0.34;
const float lowBat = 1.2;
const float fullBat = 1.62;

#define CE_PIN 2
#define CSN_PIN 4
#define MOTOR1 3
#define MOTOR2 5
#define MOTOR3 6
#define MOTOR4 9
#define VSENSOR_PIN A0

#define KP 1.2
#define KI 0.02
#define KD 0.8

#define INTEGRAL_LIMIT 400.0 // Maximum integral accumulation
#define ANGLE_LIMIT 45.0	 // Maximum angle in degrees
#define ALPHA 0.98 // Weight for gyro data (0.98 = 98% gyro, 2% accel)

unsigned long previousTime = 0;
float deltaTime = 0;

RF24 radio(CE_PIN, CSN_PIN);
const byte address[][6] = {"98830", "66310"};
MPU6050 mpu;
Servo esc1, esc2, esc3, esc4;

int joystick[4];
float batt[2];

float rollAngle = 0, pitchAngle = 0; // Filtered angles (degrees)
float rollRate = 0, pitchRate = 0;	 // Gyro rates (degrees/sec)
float rollAccel = 0, pitchAccel = 0; // Accelerometer angles (degrees)

float errorRoll = 0, errorPitch = 0;
float previousErrorRoll = 0, previousErrorPitch = 0;
float integralRoll = 0, integralPitch = 0;

float verticalAccel = 0;          // Vertical acceleration (m/s²)
float verticalVelocity = 0;       // Vertical velocity (m/s)
float estimatedHeight = 0;        // Estimated height (meters)
float baselineAccel = 0;          // Baseline acceleration for calibration
bool heightCalibrated = false;    // Calibration flag
float heightData[2];              // Array to transmit height data [height, velocity]

#define ACCEL_SCALE 16384.0       // LSB/g for ±2g range
#define GRAVITY 9.81              // m/s²
#define HEIGHT_FILTER_ALPHA 0.85  // Complementary filter for height
#define MIN_ACCEL_THRESHOLD 0.1   // Minimum acceleration threshold (m/s²)

void heightCalc() {
    // Get raw accelerometer data
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Convert raw Z-axis acceleration to m/s²
    float rawVerticalAccel = (az / ACCEL_SCALE) * GRAVITY;
    
    // Calibrate baseline if not done yet
    if (!heightCalibrated) {
        calibrateHeightBaseline();
        return;
    }
    
    // Remove gravity and baseline offset
    verticalAccel = rawVerticalAccel - baselineAccel;
    
    // Apply low-pass filter to reduce noise
    static float filteredAccel = 0;
    filteredAccel = HEIGHT_FILTER_ALPHA * filteredAccel + (1.0 - HEIGHT_FILTER_ALPHA) * verticalAccel;
    
    // Apply dead zone to eliminate small noise
    if (abs(filteredAccel) < MIN_ACCEL_THRESHOLD) {
        filteredAccel = 0;
    }
    
    // Integrate acceleration to get velocity
    verticalVelocity += filteredAccel * deltaTime;
    
    // Apply velocity damping to prevent drift
    verticalVelocity *= 0.999;
    
    // Integrate velocity to get height
    estimatedHeight += verticalVelocity * deltaTime;
    
    // Prevent negative height (ground level)
    if (estimatedHeight < 0) {
        estimatedHeight = 0;
        verticalVelocity = 0;
    }
    
    // Prepare data for transmission
    heightData[0] = estimatedHeight;           // Height in meters
    heightData[1] = verticalVelocity;          // Vertical velocity in m/s
    
    // Transmit height data
    transmitHeightData();
    
    // Debug output
    Serial.print("Height: ");
    Serial.print(estimatedHeight, 2);
    Serial.print("m, Velocity: ");
    Serial.print(verticalVelocity, 2);
    Serial.print("m/s, Accel: ");
    Serial.print(filteredAccel, 2);
    Serial.println("m/s²");
}

void calibrateHeightBaseline() {
    static int calibrationSamples = 0;
    static float accelSum = 0;
    const int CALIBRATION_SAMPLES = 200;
    
    // Get raw accelerometer data
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Convert raw Z-axis acceleration to m/s²
    float rawVerticalAccel = (az / ACCEL_SCALE) * 9.81;
    
    accelSum += rawVerticalAccel;
    calibrationSamples++;
    
    if (calibrationSamples >= CALIBRATION_SAMPLES) {
        baselineAccel = accelSum / calibrationSamples;
        heightCalibrated = true;
        
        Serial.print("Height calibration complete. Baseline: ");
        Serial.print(baselineAccel, 4);
        Serial.println(" m/s²");
        
        // Reset height tracking variables
        estimatedHeight = 0;
        verticalVelocity = 0;
        verticalAccel = 0;
    } else {
        Serial.print("Height calibration: ");
        Serial.print((calibrationSamples * 100) / CALIBRATION_SAMPLES);
        Serial.println("%");
    }
}

void transmitHeightData() {
    // Stop listening to transmit data
    radio.stopListening();
    
    // Switch to transmit pipe
    radio.openWritingPipe(address[1]);
    
    // Transmit height data
    bool result = radio.write(&heightData, sizeof(heightData));
    
    if (result) {
        Serial.println("Height data transmitted successfully");
    } else {
        Serial.println("Height data transmission failed");
    }
    
    // Switch back to listening mode
    radio.openReadingPipe(0, address[0]);
    radio.startListening();
}

void resetHeightTracking() {
    // Function to reset height tracking (call when drone lands or resets)
    estimatedHeight = 0;
    verticalVelocity = 0;
    verticalAccel = 0;
    heightCalibrated = false;
    
    Serial.println("Height tracking reset");
}

void setup()
{
	Serial.begin(115200);

	// Initialize radio
	radio.begin();
	radio.openReadingPipe(0, address[0]);
	radio.setPALevel(RF24_PA_MIN);

	// Initialize I2C and MPU6050
	Wire.begin();
	mpu.initialize();

	// Check if MPU6050 is connected
	if (mpu.testConnection())
	{
		Serial.println("MPU6050 connected successfully");
	}
	else
	{
		Serial.println("MPU6050 connection failed");
	}

	// Calibrate gyroscope (let it settle)
	calibrateGyro();

	// Initialize ESCs
	esc1.attach(MOTOR1);
	esc2.attach(MOTOR2);
	esc3.attach(MOTOR3);
	esc4.attach(MOTOR4);

	// Initialize ESCs to minimum throttle
	initializeESCs();

	previousTime = millis();

	// Calibrate gyroscope (let it settle)
	calibrateGyro();
		
	Serial.println("Setup complete");
}

void loop()
{
	// Calculate delta time for integration
	unsigned long currentTime = millis();
	deltaTime = (currentTime - previousTime) / 1000.0; // Convert to seconds
	previousTime = currentTime;

	// Ensure reasonable deltaTime (prevent division issues)
	if (deltaTime > 0.1)
		deltaTime = 0.01; // Cap at 100ms

	heightCalc();

	delay(5);
	radio.startListening();

	if (radio.available())
	{
		voltRead();
		Serial.println("Radio available");

		while (radio.available())
		{
			radio.read(&joystick, sizeof(joystick));

			// Map joystick inputs
			int throttle = map(joystick[0], 0, 255, 1000, 2000);
			float rollTarget = map(joystick[1], 0, 255, -30, 30);  // Target angle in degrees
			float pitchTarget = map(joystick[2], 0, 255, -30, 30); // Target angle in degrees
			int yawInput = map(joystick[3], 0, 255, -500, 500);

			// Get sensor data and apply complementary filter
			getMPU6050DataWithFilter();

			// Apply PID control with improved algorithm
			applyImprovedPIDControl(throttle, rollTarget, pitchTarget, yawInput);

			// Debug output
			Serial.print("Roll: ");
			Serial.print(rollAngle, 2);
			Serial.print("° Target: ");
			Serial.print(rollTarget, 2);
			Serial.print("° Throttle: ");
			Serial.println(throttle);
		}

		delay(5);
		radio.stopListening();

		// Battery monitoring and transmission
		transmitBatteryData();
	}
}

void calibrateGyro()
{
	Serial.println("Calibrating gyroscope... Keep drone still!");

	// Take 1000 samples to establish zero point
	long sumGx = 0, sumGy = 0, sumGz = 0;
	for (int i = 0; i < 1000; i++)
	{
		int16_t ax, ay, az, gx, gy, gz;
		mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		sumGx += gx;
		sumGy += gy;
		sumGz += gz;
		delay(3);
	}

	// Calculate offsets (these should be subtracted from future readings)
	// For simplicity, we're not storing offsets in this example
	// In a full implementation, you'd store these and subtract them

	Serial.println("Gyroscope calibration complete");
}

void initializeESCs()
{
	Serial.println("Initializing ESCs...");

	// Send minimum throttle to all ESCs
	esc1.writeMicroseconds(1000);
	esc2.writeMicroseconds(1000);
	esc3.writeMicroseconds(1000);
	esc4.writeMicroseconds(1000);

	delay(2000); // Wait for ESCs to recognize the signal
	Serial.println("ESCs initialized");
}

void getMPU6050DataWithFilter()
{
	int16_t ax, ay, az, gx, gy, gz;
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

	// Convert gyroscope data to degrees per second
	rollRate = gx / 131.0;
	pitchRate = gy / 131.0;

	// Convert accelerometer data to angles (in degrees)
	rollAccel = atan2(ay, az) * 180.0 / PI;
	pitchAccel = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

	// Apply complementary filter
	// Integrate gyro data and combine with accelerometer
	rollAngle = ALPHA * (rollAngle + rollRate * deltaTime) + (1.0 - ALPHA) * rollAccel;
	pitchAngle = ALPHA * (pitchAngle + pitchRate * deltaTime) + (1.0 - ALPHA) * pitchAccel;

	// Constrain angles to reasonable limits
	rollAngle = constrain(rollAngle, -ANGLE_LIMIT, ANGLE_LIMIT);
	pitchAngle = constrain(pitchAngle, -ANGLE_LIMIT, ANGLE_LIMIT);
}

void applyImprovedPIDControl(int throttle, float rollTarget, float pitchTarget, int yawInput)
{
	// Calculate errors (target - actual)
	errorRoll = rollTarget - rollAngle;
	errorPitch = pitchTarget - pitchAngle;

	// Accumulate errors for integral term
	integralRoll += errorRoll * deltaTime;
	integralPitch += errorPitch * deltaTime;

	// Apply integral windup protection
	integralRoll = constrain(integralRoll, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
	integralPitch = constrain(integralPitch, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

	// Reset integral if error changes sign (prevents windup)
	if ((errorRoll > 0 && previousErrorRoll < 0) || (errorRoll < 0 && previousErrorRoll > 0))
	{
		integralRoll *= 0.5; // Reduce integral by half
	}
	if ((errorPitch > 0 && previousErrorPitch < 0) || (errorPitch < 0 && previousErrorPitch > 0))
	{
		integralPitch *= 0.5; // Reduce integral by half
	}

	// Calculate derivative term (rate of error change)
	float derivativeRoll = (errorRoll - previousErrorRoll) / deltaTime;
	float derivativePitch = (errorPitch - previousErrorPitch) / deltaTime;

	// Apply derivative filter to reduce noise
	static float lastDerivativeRoll = 0, lastDerivativePitch = 0;
	derivativeRoll = 0.7 * lastDerivativeRoll + 0.3 * derivativeRoll;
	derivativePitch = 0.7 * lastDerivativePitch + 0.3 * derivativePitch;
	lastDerivativeRoll = derivativeRoll;
	lastDerivativePitch = derivativePitch;

	// Calculate PID output
	float pidRoll = (KP * errorRoll) + (KI * integralRoll) + (KD * derivativeRoll);
	float pidPitch = (KP * errorPitch) + (KI * integralPitch) + (KD * derivativePitch);

	// Store current errors for next iteration
	previousErrorRoll = errorRoll;
	previousErrorPitch = errorPitch;

	// Constrain PID output to reasonable limits
	pidRoll = constrain(pidRoll, -500, 500);
	pidPitch = constrain(pidPitch, -500, 500);

	// Apply motor mixing for X configuration
	// Only apply stabilization if throttle is above minimum
	if (throttle > 1100)
	{
		int motor1Speed = constrain(throttle + pidRoll + pidPitch - yawInput, 1000, 2000);
		int motor2Speed = constrain(throttle + pidRoll - pidPitch + yawInput, 1000, 2000);
		int motor3Speed = constrain(throttle - pidRoll - pidPitch - yawInput, 1000, 2000);
		int motor4Speed = constrain(throttle - pidRoll + pidPitch + yawInput, 1000, 2000);

		esc1.writeMicroseconds(motor1Speed);
		esc2.writeMicroseconds(motor2Speed);
		esc3.writeMicroseconds(motor3Speed);
		esc4.writeMicroseconds(motor4Speed);
	}
	else
	{
		// Throttle too low - stop all motors and reset integrals
		esc1.writeMicroseconds(1000);
		esc2.writeMicroseconds(1000);
		esc3.writeMicroseconds(1000);
		esc4.writeMicroseconds(1000);

		// Reset integral terms when not flying
		integralRoll = 0;
		integralPitch = 0;

    // Reset height tracking when not flying
    resetHeightTracking();
	}
}

void voltRead()
{
	int i;
	float batt_volt = 0;

	for (i = 0; i < 100; i++)
	{
		batt_volt += ((analogRead(VSENSOR_PIN) * (4.49 / 1023.0)) * 4);
		delay(1); // Reduced delay for faster sampling
	}

	batt_volt = batt_volt / i;

	Serial.print("Battery: ");
	Serial.print(batt_volt, 2);
	Serial.print("V ");
	Serial.print(((batt_volt - lowBat) / (fullBat - lowBat)) * 100);
	Serial.println("%");
}

void transmitBatteryData()
{
	int i;
	batt[0] = 0;

	for (i = 0; i < 50; i++) // Reduced samples for faster execution
	{
		batt[0] += ((analogRead(VSENSOR_PIN) * (4.49 / 1023.0)) * 4);
		delay(2);
	}

	batt[0] = batt[0] / i;
	batt[1] = ((batt[0] - lowBat) / (fullBat - lowBat)) * 100;

	radio.write(&batt, sizeof(batt));
	delay(50); // Reduced delay
}
