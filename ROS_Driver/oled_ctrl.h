// <<<<<<<<<<=== === ===SSD1306: 0x3C=== === ===>>>>>>>>>>
// 0.91inch OLED
bool screenDefaultMode = true;

unsigned long currentTimeMillis = millis();
unsigned long lastTimeMillis = millis();

// default
String screenLine_0;
String screenLine_1;

// custom
String customLine_0;
String customLine_1;

// #include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels
#define OLED_RESET -1     // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS \
  0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// init oled ctrl functions.
void init_oled() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  // use larger text so we only have 2 lines on a 32px-high display
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.display();
}

// Updata all data and flash the screen.
void oled_update() {
  display.clearDisplay();
  display.setCursor(0, 0);

  // print two larger lines (font height x2 -> ~16px per line on default font)
  display.println(screenLine_0);
  display.println(screenLine_1);

  display.display();
}

// dev info update on oled.
void oledInfoUpdate() {
  currentTimeMillis = millis();
  if (currentTimeMillis - lastTimeMillis > 10000) {
    inaDataUpdate();
    lastTimeMillis = currentTimeMillis;
  } else {
    return;
  }
  if (!screenDefaultMode) {
    return;
  }
  // output voltage/current in V and A
  screenLine_1 =
      String(loadVoltage_V, 1) + "/" + String(current_mA / 1000.0, 1);
  oled_update();
}

// oled ctrl.
void oledCtrl(byte inputLineNum, String inputMegs) {
  screenDefaultMode = false;
  switch (inputLineNum) {
    case 0:
      customLine_0 = inputMegs;
      break;
    case 1:
      customLine_1 = inputMegs;
      break;
  }
  display.clearDisplay();
  display.setCursor(0, 0);

  // show only two custom lines with larger text
  display.println(customLine_0);
  display.println(customLine_1);

  display.display();
}

// set oled as default.
void setOledDefault() {
  screenDefaultMode = true;
  // force update
  lastTimeMillis = 0;
  oledInfoUpdate();
}