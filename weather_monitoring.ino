#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// Define pins
#define DHTPIN 2
#define DHTTYPE DHT11
#define BUZZER_PIN 8
#define IR_SENSOR_PIN 3

// ===== BUZZER SETTINGS =====
#define BUZZER_FREQUENCY 2500
#define BUZZER_DURATION 800

// ===== THRESHOLD SETTINGS =====
#define TEMP_THRESHOLD 35.0
#define HUMID_THRESHOLD 80.0
#define WIND_THRESHOLD 1.1

// ===== TIMING SETTINGS =====
#define UPDATE_INTERVAL 2000
#define WARNING_INTERVAL 1000
#define COOLDOWN_INTERVAL 1000

// ===== WIND SPEED SETTINGS =====
#define WIND_CALC_INTERVAL 1000
#define WIND_CAL_A 8.75
#define WIND_CAL_B 50
#define MAX_WIND_KMH 300.0

// ===== I2C ERROR DETECTION SETTINGS =====
#define I2C_RETRY_DELAY 100
#define I2C_MAX_RETRIES 3
#define I2C_CHECK_INTERVAL 5000

// Initialize LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Initialize DHT sensor
DHT dht(DHTPIN, DHTTYPE);

// Custom hollow degree symbol
byte hollowDegree[8] = {
  0b00110,
  0b01001,
  0b01001,
  0b00110,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

// Variables
float temperature = 0;
float humidity = 0;
unsigned long previousMillis = 0;
const long interval = UPDATE_INTERVAL;

// State variables
unsigned long warningTimer = 0;
unsigned long cooldownTimer = 0;
int warningDisplayMode = 0;
bool isWarningDisplayed = false;
bool displayNeedsUpdate = true;
bool buzzerSounded = false;
bool isInCooldown = false;
unsigned long buzzerStartTime = 0;
bool isBuzzerActive = false;

// ===== WIND SPEED VARIABLES =====
volatile unsigned long pulseCount = 0;
unsigned long windTimer = 0;
float windSpeedKmh = 0;

// ===== DISPLAY MODE =====
enum DisplayMode { MODE_NONE, MODE_READINGS, MODE_WARNING };
DisplayMode currentDisplayMode = MODE_NONE;

// ===== I2C ERROR DETECTION =====
unsigned long lastI2CCheck = 0;
int i2cErrorCount = 0;
bool lcdNeedsReset = false;
unsigned long lastSuccessfulLCDUpdate = 0;

// ===== WARNING STATE TRACKING =====
bool warningWasActive = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Weather Monitor Starting...");

  Wire.begin();
  Wire.setClock(100000);

  if (!initLCD()) {
    Serial.println("LCD initialization failed - will retry");
    lcdNeedsReset = true;
  }

  lcd.createChar(0, hollowDegree);

  lcd.setCursor(0, 0);
  lcd.print("Weather Monitor");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(2000);

  dht.begin();
  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(IR_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), countPulse, FALLING);

  readSensor();
  windTimer = millis();
  lastSuccessfulLCDUpdate = millis();

  showReadings();
  Serial.println("Setup complete!");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastI2CCheck >= I2C_CHECK_INTERVAL) {
    lastI2CCheck = currentMillis;
    checkI2CHealth();
  }

  if (lcdNeedsReset) {
    Serial.println("LCD needs reset - attempting recovery...");
    delay(100);
    
    if (resetLCD()) {
      lcdNeedsReset = false;
      i2cErrorCount = 0;
      Serial.println("LCD reset successful!");
      displayNeedsUpdate = true;
      currentDisplayMode = MODE_NONE;
    } else {
      Serial.println("LCD reset failed - will retry later");
      lcdNeedsReset = true;
    }
  }

  if (currentMillis - previousMillis >= interval && !isBuzzerActive && !lcdNeedsReset) {
    previousMillis = currentMillis;
    readSensor();
    displayNeedsUpdate = true;
  }

  if (currentMillis - windTimer >= WIND_CALC_INTERVAL && !lcdNeedsReset) {
    windTimer = currentMillis;
    calculateWindSpeed();
    displayNeedsUpdate = true;
  }

  if (!lcdNeedsReset) {
    checkThresholds();
  }

  if (isBuzzerActive) {
    if (currentMillis - buzzerStartTime >= BUZZER_DURATION) {
      noTone(BUZZER_PIN);
      isBuzzerActive = false;
      Serial.println("Buzzer stopped");
    }
  }

  delay(10);
}

// ===== I2C HEALTH CHECK FUNCTIONS =====

bool initLCD() {
  for (int i = 0; i < I2C_MAX_RETRIES; i++) {
    Wire.beginTransmission(0x27);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      lcd.init();
      lcd.backlight();
      lcd.clear();
      Serial.println("LCD initialized successfully");
      return true;
    }
    
    Serial.print("LCD init attempt ");
    Serial.print(i + 1);
    Serial.print(" failed with error: ");
    Serial.println(error);
    delay(I2C_RETRY_DELAY);
  }
  return false;
}

void checkI2CHealth() {
  Wire.beginTransmission(0x27);
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    i2cErrorCount++;
    Serial.print("I2C error detected! Error: ");
    Serial.print(error);
    Serial.print(" (Count: ");
    Serial.print(i2cErrorCount);
    Serial.println(")");
    
    if (i2cErrorCount >= 3) {
      Serial.println("Multiple I2C errors - resetting LCD");
      lcdNeedsReset = true;
      i2cErrorCount = 0;
    }
  } else {
    if (i2cErrorCount > 0) {
      i2cErrorCount--;
      Serial.print("I2C healthy - error count reduced to: ");
      Serial.println(i2cErrorCount);
    }
  }
}

bool resetLCD() {
  Serial.println("=== LCD RESET SEQUENCE STARTING ===");
  
  detachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN));
  
  noTone(BUZZER_PIN);
  isBuzzerActive = false;
  
  Wire.end();
  delay(100);
  Wire.begin();
  Wire.setClock(100000);
  delay(100);
  
  bool success = false;
  for (int i = 0; i < 3; i++) {
    Wire.beginTransmission(0x27);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      lcd.init();
      lcd.backlight();
      lcd.clear();
      lcd.createChar(0, hollowDegree);
      lcd.setCursor(0, 0);
      lcd.print("LCD Recovered!");
      lcd.setCursor(0, 1);
      lcd.print("Attempt: ");
      lcd.print(i + 1);
      delay(1000);
      lcd.clear();
      
      success = true;
      break;
    }
    
    Serial.print("Reset attempt ");
    Serial.print(i + 1);
    Serial.print(" failed, error: ");
    Serial.println(error);
    delay(500);
  }
  
  if (success) {
    currentDisplayMode = MODE_NONE;
    displayNeedsUpdate = true;
    i2cErrorCount = 0;
    lastSuccessfulLCDUpdate = millis();
    Serial.println("=== LCD RESET SUCCESSFUL ===");
  } else {
    Serial.println("=== LCD RESET FAILED ===");
  }
  
  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), countPulse, FALLING);
  
  return success;
}

void checkThresholds() {
  unsigned long currentMillis = millis();

  bool highTemp = (temperature > TEMP_THRESHOLD);
  bool highHumid = (humidity > HUMID_THRESHOLD);
  bool highWind = (windSpeedKmh > WIND_THRESHOLD);
  bool isWarningCondition = (highTemp || highHumid || highWind);

  if (isWarningCondition && !warningWasActive) {
    buzzerSounded = false;
    Serial.println("Warning state entered - buzzer reset");
  }
  warningWasActive = isWarningCondition;

  if (isInCooldown) {
    if (currentMillis - cooldownTimer >= COOLDOWN_INTERVAL) {
      isInCooldown = false;
      isWarningDisplayed = false;
      buzzerSounded = false;
      displayNeedsUpdate = true;
      Serial.println("Cooldown ended");
    } else {
      if (displayNeedsUpdate) {
        showReadings();
        displayNeedsUpdate = false;
      }
      return;
    }
  }

  if (isWarningCondition) {
    if (highTemp && highHumid && highWind) {
      warningDisplayMode = 6;
    } else if (highHumid && highWind) {
      warningDisplayMode = 5;
    } else if (highTemp && highWind) {
      warningDisplayMode = 4;
    } else if (highTemp && highHumid) {
      warningDisplayMode = 2;
    } else if (highWind) {
      warningDisplayMode = 3;
    } else if (highTemp) {
      warningDisplayMode = 1;
    } else if (highHumid) {
      warningDisplayMode = 0;
    }

    if (currentMillis - warningTimer >= WARNING_INTERVAL) {
      warningTimer = currentMillis;
      isWarningDisplayed = !isWarningDisplayed;
      displayNeedsUpdate = true;

      if (isWarningDisplayed) {
        buzzerSounded = false;
      }
    }

    if (displayNeedsUpdate) {
      if (isWarningDisplayed) {
        showWarning(warningDisplayMode);
      } else {
        showReadings();
      }
      displayNeedsUpdate = false;
    }

    if (isWarningDisplayed && !buzzerSounded) {
      Serial.println("Starting buzzer!");
      tone(BUZZER_PIN, BUZZER_FREQUENCY, BUZZER_DURATION);
      buzzerStartTime = currentMillis;
      isBuzzerActive = true;
      buzzerSounded = true;
    }

  } else {
    if (displayNeedsUpdate || currentDisplayMode != MODE_READINGS) {
      showReadings();
      displayNeedsUpdate = false;
      isWarningDisplayed = false;
      buzzerSounded = false;
    }

    if (isBuzzerActive) {
      noTone(BUZZER_PIN);
      isBuzzerActive = false;
    }

    warningTimer = millis();

    if (isInCooldown == false && isWarningCondition == false) {
      isInCooldown = true;
      cooldownTimer = currentMillis;
      Serial.println("Cooldown started");
    }
  }
}

void countPulse() {
  pulseCount++;
}

void calculateWindSpeed() {
  noInterrupts();
  unsigned long pulses = pulseCount;
  pulseCount = 0;
  interrupts();

  windSpeedKmh = (pulses * WIND_CAL_A) / WIND_CAL_B;

  if (windSpeedKmh > MAX_WIND_KMH || isnan(windSpeedKmh)) {
    windSpeedKmh = MAX_WIND_KMH;
  }
}

void readSensor() {
  float newHumidity = dht.readHumidity();
  float newTemperature = dht.readTemperature();

  if (!isnan(newHumidity) && !isnan(newTemperature)) {
    temperature = newTemperature;
    humidity = newHumidity;

    Serial.print("T: ");
    Serial.print(temperature);
    Serial.print("C, H: ");
    Serial.print(humidity);
    Serial.println("%");
  } else {
    Serial.println("Sensor read error - keeping last good values");
  }
}

// ===== DISPLAY FUNCTIONS =====

void showReadings() {
  if (lcdNeedsReset) {
    return;
  }
  
  detachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN));
  
  bool modeChanged = (currentDisplayMode != MODE_READINGS);

  char tBuf[12];
  char hBuf[12];
  char wBuf[12];
  dtostrf(temperature, 4, 1, tBuf);
  dtostrf(humidity, 4, 1, hBuf);
  dtostrf(windSpeedKmh, 4, 1, wBuf);

  if (modeChanged) {
    lcd.clear();
    delay(5);
  }

  lcd.setCursor(0, 0);
  delay(1);
  lcd.print("T:");
  lcd.print(tBuf);
  lcd.write(0);
  lcd.print("C H:");
  lcd.print(hBuf);
  lcd.print("%");

  lcd.setCursor(0, 1);
  delay(1);
  lcd.print("WS:");
  lcd.print(wBuf);
  lcd.print("km/h     ");

  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), countPulse, FALLING);

  currentDisplayMode = MODE_READINGS;
  lastSuccessfulLCDUpdate = millis();
}

void showWarning(int warningType) {
  if (lcdNeedsReset) {
    return;
  }
  
  detachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN));
  
  bool modeChanged = (currentDisplayMode != MODE_WARNING);

  if (modeChanged) {
    lcd.clear();
    delay(5);
  }

  lcd.setCursor(0, 0);
  delay(1);
  lcd.print("!!! WARNING !!! ");

  lcd.setCursor(0, 1);
  delay(1);
  switch (warningType) {
    case 0:
      lcd.print("HIGH HUMIDITY   ");
      break;
    case 1:
      lcd.print("HIGH TEMPERATURE");
      break;
    case 2:
      lcd.print("HIGH T & H      ");
      break;
    case 3:
      lcd.print("HIGH WIND SPEED ");
      break;
    case 4:
      lcd.print("HIGH T & WIND   ");
      break;
    case 5:
      lcd.print("HIGH H & WIND   ");
      break;
    case 6:
      lcd.print("HIGH T, H & WIND");
      break;
  }

  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), countPulse, FALLING);

  currentDisplayMode = MODE_WARNING;
  lastSuccessfulLCDUpdate = millis();
}