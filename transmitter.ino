#include <Adafruit_SSD1306.h>
#include <splash.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 5
#define CSN_PIN 6
#define JOY_THROTTLE A0
#define JOY_ROLL A1
#define JOY_PITCH A2
#define JOY_YAW A3
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

RF24 radio(CE_PIN, CSN_PIN);
const byte address[][6] = {"98830", "66310"};
int joystick[4];

void setup()
{
	Serial.begin(9600);
	radio.begin();
	radio.openWritingPipe(address);
	radio.setPALevel(RF24_PA_MIN);
	if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
	{
		Serial.println("OLED allocation failed");
	}
	beautifulInitialization();

	display.clearDisplay();
  display.setTextSize(2);
	display.setCursor(0, 16);
	display.println(" Ready to\ntake off!!");
	display.display();
}

// Beautiful animated initialization for SSD1306 OLED
void beautifulInitialization() {
  display.clearDisplay();
  
  // Phase 1: Animated dots appearing
  for(int phase = 0; phase < 4; phase++) {
    display.clearDisplay();
    
    // Draw expanding circles
    for(int i = 0; i <= phase; i++) {
      int x = 32 + i * 16;
      int y = 20;
      display.fillCircle(x, y, 3 + (phase - i), SSD1306_WHITE);
    }
    
    // Pulsing center logo/icon
    int pulseSize = 2 + sin(millis() * 0.01) * 2;
    display.fillRect(60, 8, 8, 8, SSD1306_WHITE);
    display.fillRect(58, 10, 12, 4, SSD1306_WHITE);
    
    display.display();
    delay(300);
  }
  
  // Phase 2: Progress bar animation
  for(int progress = 0; progress <= 100; progress += 5) {
    display.clearDisplay();
    
    // Title with modern font effect
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(35, 5);
    display.println("SYSTEM");
    
    // Stylized progress bar with rounded edges
    int barWidth = map(progress, 0, 100, 0, 96);
    
    // Background bar
    display.drawRect(16, 25, 96, 8, SSD1306_WHITE);
    
    // Progress fill with animated segments
    for(int i = 0; i < barWidth; i += 4) {
      if((i / 4) % 2 == 0) {
        display.fillRect(17 + i, 26, min(3, barWidth - i), 6, SSD1306_WHITE);
      }
    }
    
    // Percentage display
    display.setTextSize(1);
    display.setCursor(55, 40);
    display.print(progress);
    display.println("%");
    
    // Animated corner decorations
    int corner = (progress / 10) % 4;
    display.fillTriangle(0, 0, 8, 0, 0, 8, SSD1306_WHITE);
    if(corner >= 1) display.fillTriangle(120, 0, 128, 0, 128, 8, SSD1306_WHITE);
    if(corner >= 2) display.fillTriangle(128, 56, 128, 64, 120, 64, SSD1306_WHITE);
    if(corner >= 3) display.fillTriangle(0, 64, 8, 64, 0, 56, SSD1306_WHITE);
    
    display.display();
    delay(100);
  }
  
  // Phase 3: Success animation
  display.clearDisplay();
  
  // Final hold
  delay(800);
  
  display.clearDisplay();
  display.display();
}

void loop()
{
	float VIN;  //Battery voltage
  float HGT;  //Height
	delay(5);
	radio.stopListening();
	joystick[0] = analogRead(JOY_THROTTLE) / 4;
	joystick[1] = analogRead(JOY_ROLL) / 4;
	joystick[2] = analogRead(JOY_PITCH) / 4;
	joystick[3] = analogRead(JOY_YAW) / 4;
	radio.write(&joystick, sizeof(joystick));
	delay(5);
	radio.startListening();
	
  while (!radio.available())
	{
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("NO SIGNAL");
    display.display();
	}

	radio.read(&VIN, sizeof(VIN));
	radio.read(&HGT, sizeof(HGT));
	display.clearDisplay();
	display.setCursor(0, 16);

  display.setTextSize(1);
  display.print("Battery:");
  display.setTextSize(2);
  display.print(VIN);
  display.println("V");

  display.setTextSize(1);
  display.print("\nHeight:");
  display.setTextSize(2);
  display.print(HGT);
  display.println("m\n");

	display.display();
}
