/*
 * 
 * External Libraries Requirements:
 *   LiquidCrystal_I2C ~ Arduino-LiquidCrystal-I2C-library-master ~ Copied from zip: Arduino-LiquidCrystal-I2C-library-master
 *   Keypad            ~ https://playground.arduino.cc/Code/Keypad/ ~ Installed via Manage Libraries: "Keypad"
 *   DS3231            ~ DS3231 ~ http://www.rinkydinkelectronics.com/ ~ Copied from zip: DS3231
 *   AccelStepper      ~ http://www.airspayce.com/mikem/arduino/AccelStepper/ ~ Installed via Manage Libraries: AccelStepper by Mike Mcauley Version 1.59.0
 *
 *
 * Change Log:
 *  Version 2.11: 2020-04-28 Milan
 *     LiquidCrystal_I2C Pointer Fix
 *  Version 2.1: 2020-04-28 Milan
 *     Added init_lcd autodetect
 *  Version 2.0: 2020-01-11 Milan
 *     Added Change Log and External libraries list
 *     Renamed: mixerPumpPin -> TankMixerPin
 *     Renamed: VACUUM_PIN   -> DoserAirPumpPin
 *     Renamed: motor_en_pin -> MotorEnPin
 *     mixer_run(255); and mixer_run(0); was PWM speed controller, now on/off
 *        mixer_run(spd) -> mixer_run_old(spd); old implementation kept for reference
 *        mixer_run(255) -> mixer_run();   which now uses digitalWrite(PIN, HIGH); rather than analogWrite setting PWM
 *        mixer_run(0)   -> mixer_stop();  which now uses digitalWrite(PIN, LOW);
 *     Added:  delay_loop to join few identical code snippets  
 *  Version 1.0: legacy code
 *  
 */
 
#define VOLTAGE_LIMIT 12.0 //set battery voltage threshhold in volts
//Values caps:
#define VOLUME_MAX 9999
#define VOLUME_MIN 0
#define TIMES_A_DAY_MAX 9
#define TIMES_A_DAY_MIN 0
#define PUMP_RUN_MAX 99
#define PUMP_RUN_MIN 0

#define IDLE_ROTATIONS 4 //number of idle rotations before counting the dosage

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <DS3231.h>
#include <EEPROM.h>
#include <AccelStepper.h>
#include <avr/io.h>

#define DoserAirPumpPin 2

#define _NORM 1
#define _SETUP 2
#define _ERROR 3
#define LCDWORKTIME 50000
#define BUTTONRELEASETIME 500
#define MLPERROTATION 1.45  // 6.4 mL for Four pump heads or 1.45mL for 1 pump head???  3.15 or 4.7 

#define EEPROM_START_TIME 10
#define EEPROM_TIMES 30
#define EEPROM_VOLUME 40
#define EEPROM_PUMP_TIME 50
#define EEPROM_ERROR 60


int TankMixerPin = 3;
const byte ROWS = 4; //four rows
const byte COLS = 3; //three columns
char keys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

byte rowPins[ROWS] = {10, 9, 8, 7};
byte colPins[COLS] = {6, 5, 4};

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

// List of possible LCD addresses used to initialise LCD
const uint8_t lcd_addresses[] = { 0x27, 0x3F }; 
#define NUMLCDCOLS 20
#define NUMLCDROWS  4


uint8_t scanI2C() {
  int i;
  byte error;
  uint8_t address;
  
  Serial.begin(9600);
  
  delay(250);
  
  
  Wire.begin();
  
  
  for(i = 0; i < sizeof(lcd_addresses); i++) {
    address = lcd_addresses[i];
    
    Wire.beginTransmission(address);
    
    
    error = Wire.endTransmission();
    
    if (error == 0) {
      if (address<16) 
      Serial.print(address, HEX);
      return address;
    } 
    else if (error == 4) 
    {
      
      if (address < 16) 
        Serial.println(address,HEX);
    } 
  }
  if (i >= sizeof(lcd_addresses)) {
    // None of the selected addresses recognised as LCD
    delay(5000);
  } 
  return lcd_addresses[0];
} // scanI2C

LiquidCrystal_I2C *lcd;

unsigned long stepsPerRevolution = 200;
AccelStepper motor(1, A1, A0);
int MotorEnPin  = A2;

DS3231  rtc(SDA, SCL);
Time  t, startTime, intervalTime;
unsigned long loopTimer = 0;
unsigned long ledTimer = 0;
unsigned long buttonLastHit = 0;
int mode = _NORM;

byte val;
int curs = 0;
int pointer = 0;
int vals[4];
int volume = 0;
int timesAday = 0;
int pumpRunTime = 0;
int dose = 0;

float coef = 0.0153;
float getBatteryVoltage() {
  return analogRead(A6) * coef;
}
void(* reset) (void) = 0;


void delay_loop(unsigned long ms, int n) {
  for (int i=0;i<n;i++) {
    delay(ms);
    lcd->print(".");  
  }
  lcd->clear();
}

void setup()
{
  // Scan for LCD 
  uint8_t lcdAddress = scanI2C(); 
  // Initialise LCD
  lcd = new LiquidCrystal_I2C(lcdAddress, NUMLCDCOLS, NUMLCDROWS);
  
  motor.setMaxSpeed(stepsPerRevolution);
  motor.setAcceleration(100);
  pinMode(TankMixerPin, OUTPUT);
  pinMode(DoserAirPumpPin, OUTPUT);
  pinMode(MotorEnPin, OUTPUT);
  shut_down_everything();
  rtc.begin();
  lcd->begin();
  lcd->backlight();
  lcd->print(F("Launching"));

  delay_loop(500, 3);

  clear_vals();
  if (EEPROM.read(EEPROM_START_TIME - 1) == 0) EEPROM.get(EEPROM_START_TIME, startTime);
  if (EEPROM.read(EEPROM_VOLUME - 1) == 0)       EEPROM.get(EEPROM_VOLUME, volume);
  if (EEPROM.read(EEPROM_TIMES - 1) == 0)       EEPROM.get(EEPROM_TIMES, timesAday);
  if (EEPROM.read(EEPROM_PUMP_TIME - 1) == 0)       EEPROM.get(EEPROM_PUMP_TIME, pumpRunTime);
  calculate();

}

void loop()
{
  if (millis() - loopTimer > 50) {
    if (!check_values()) criticalError();

    t = rtc.getTime();
        switch (mode) {
      case _NORM: {
          showTimeVals();
          if (timesAday != 0) {
            dosing_handler();
          }
          char key = keypad.getKey();
          if (key) {
            LCD_on(); ledTimer = millis();
            if ( key == '#') {
              mode = _SETUP;
              LCD_on();
              key = NULL;
              lcd->clear();
              buttonLastHit = millis();
            }
          }
          else {
            if (millis() - ledTimer > LCDWORKTIME) {
              LCD_off();
              ledTimer = millis();
            }
          }
        }
        break;
      case _SETUP:
        setup_menu();
        break;

    }
    loopTimer = millis();
  }
}

bool check_values() {
  if (volume < VOLUME_MIN || volume > VOLUME_MAX) return 0;
  if (timesAday < TIMES_A_DAY_MIN || timesAday > TIMES_A_DAY_MAX) return 0;
  if (pumpRunTime < PUMP_RUN_MIN || pumpRunTime > PUMP_RUN_MAX ) return 0;
  return 1;
}

void criticalError() {
  if (EEPROM.read(EEPROM_ERROR) != 0) {
    EEPROM.write(EEPROM_ERROR, 0);
    reset();
  }
  else {
    EEPROM.write(EEPROM_ERROR, 128);
    shut_down_everything();
    lcd->clear();
    lcd->home();
    lcd->print(F("Critical error, shutting down"));
    while (1);
  }
}
void saveToEEPROM() {
  EEPROM.write(EEPROM_VOLUME - 1, 0);
  EEPROM.put(EEPROM_VOLUME, volume);

  EEPROM.write(EEPROM_START_TIME - 1, 0);
  EEPROM.put(EEPROM_START_TIME, startTime);

  EEPROM.write(EEPROM_TIMES - 1, 0);
  EEPROM.put(EEPROM_TIMES, timesAday);
}

Time plus(Time one, Time two) {
  int ff = (one.hour + two.hour) * 60 + one.min + two.min;

  if (ff > 1439) ff = ff - 1440;

  Time ttt;
  ttt.min = ff % 60;
  ttt.hour = (ff - ff % 60) / 60;
  return ttt;
}

void calculate() {
  if (timesAday != 0) {
    dose = volume / timesAday;
    int ff = 1440 / timesAday;
    intervalTime.min = ff % 60;
    intervalTime.hour = (ff - ff % 60) / 60;
  }
}

bool dosing_handler() {
  for (int i = 0; i < timesAday; i++) {
    if (its_time(plus(startTime, mult(intervalTime, i)))) {
      if(getBatteryVoltage() > VOLTAGE_LIMIT) do_the_dosing();
    }
  }
  return 0;
}

Time mult(Time tm, int multipl ) {
  Time dummy;
  dummy.hour = 0;
  dummy.min = 0;
  for (int i = 0; i < multipl; i++) {
    dummy.hour = plus(dummy, tm).hour;
    dummy.min = plus(dummy, tm).min;
  }
  return dummy;
}


bool its_time (Time Start) {
  if ( (t.hour * 60 + t.min) == (Start.hour * 60 + Start.min) ) return true;
  else return false;
}

void clear_vals() {
  for (int i = 0; i < 4; i++) {
    vals[i] = -1;
  }
}

void setup_menu() {
  char key = keypad.getKey();
  if ( key && millis() - buttonLastHit > BUTTONRELEASETIME)
  {
    if (key == '*') {
      curs--;
      if (curs < 0) {
        mode = _NORM;
        calculate();
        lcd->clear();
      }
    }
    else if (key == '#') {
      if ( vals[3] != -1 ) {
        lcd->clear();
        lcd->home();
        lcd->print(F("Setting updated!"));
        record(curs);
        delay(2000);
      }
      curs++;
      clear_vals();
      if (curs == 3) pointer = 3;
      else if ( curs == 4) pointer = 2;
      else pointer = 0;
      lcd->clear();
      if (curs > 4) {
        mode = _NORM;
        calculate();
      }
    }
    else if (key) {
      vals[pointer] = key - '0';
      pointer++;
      if (pointer > 3) {
        if (curs == 3) pointer = 3;
        else if (curs == 4) pointer = 2;
        else pointer = 0;
      }
    }
    buttonLastHit = millis();
  }
  valueSetMenu(curs);
}

void record(int ID) {
  switch (ID) {
    case 0:
      rtc.setTime(vals[0] * 10 + vals[1], vals[2] * 10 + vals[3], 00);
      break;
    case 1:
      volume = vals[0] * 1000 + vals[1] * 100 + vals[2] * 10 + vals[3];
      EEPROM.write(EEPROM_VOLUME - 1, 0);
      EEPROM.put(EEPROM_VOLUME, volume);
      break;
    case 2:
      startTime.hour = vals[0] * 10 + vals[1];
      startTime.min = vals[2] * 10 + vals[3];
      EEPROM.write(EEPROM_START_TIME - 1, 0);
      EEPROM.put(EEPROM_START_TIME, startTime);
      break;
    case 3:
      timesAday = vals[3];
      EEPROM.write(EEPROM_TIMES - 1, 0);
      EEPROM.put(EEPROM_TIMES, timesAday);
      break;
    case 4:
      pumpRunTime = vals[2] * 10 + vals[3];
      EEPROM.write(EEPROM_PUMP_TIME - 1, 0);
      EEPROM.put(EEPROM_PUMP_TIME, pumpRunTime);
      break;
  }
}

void valueSetMenu(int ID) {
  lcd->home();
  switch (ID) {
    case 0:
      lcd->print(F("Set time:"));
      lcd->setCursor(4, 1);
      for (int i = 0; i < 4; i++) {
        if (vals[i] != -1) lcd->print(vals[i]);
        else lcd->print("_");
        if (i == 1) lcd->print(":");
      }
      lcd->setCursor(3, 2); lcd->print(F("Example:"));
      lcd->setCursor(5, 3); ; lcd->print("08:19");
      break;
    case 1:
      lcd->print(F("Set volume:"));
      lcd->setCursor(4, 1);
      for (int i = 0; i < 4; i++) {
        if (vals[i] != -1) lcd->print(vals[i]);
        else lcd->print("_");
      }
      lcd->setCursor(3, 2); lcd->print(F("Example:"));
      lcd->setCursor(5, 3); ; lcd->print("0245");
      break;
    case 2:
      lcd->print(F("Set start time:"));
      lcd->setCursor(4, 1);
      for (int i = 0; i < 4; i++) {
        if (vals[i] != -1) lcd->print(vals[i]);
        else lcd->print("_");
        if (i == 1) lcd->print(":");
      }
      lcd->setCursor(3, 2); lcd->print(F("Example:"));
      lcd->setCursor(5, 3); ; lcd->print("18:09");
      break;
    case 3:
      lcd->print(F("Set times a day:"));
      lcd->setCursor(4, 1);
      for (int i = 3; i < 4; i++) {
        if (vals[i] != -1) lcd->print(vals[i]);
        else lcd->print("_");
      }
      lcd->setCursor(3, 2); lcd->print(F("Example:"));
      lcd->setCursor(5, 3); ; lcd->print("5");
      break;
    case 4:
      lcd->print(F("Set pump run time:"));
      lcd->setCursor(4, 1);
      for (int i = 2; i < 4; i++) {
        if (vals[i] != -1) lcd->print(vals[i]);
        else lcd->print("_");
      }
      lcd->setCursor(3, 2); lcd->print(F("Example:"));
      lcd->setCursor(5, 3); ; lcd->print("15");
      break;
  }
}


void LCD_off() {
  lcd->noBacklight();
}

void LCD_on() {
  lcd->backlight();
}

void shut_down_everything() {
  mixer_stop();  
  pump_stop();
  vacuum_stop();
}

void mixer_run_old(int spd) {
  spd =  constrain(spd, 0, 255);
  analogWrite(TankMixerPin, spd);
}

void mixer_run() {
  digitalWrite(TankMixerPin, HIGH);
}

void mixer_stop() {
  digitalWrite(TankMixerPin, LOW);
}

void vacuum_run() {
  digitalWrite(DoserAirPumpPin, HIGH);
}

void vacuum_stop() {
  digitalWrite(DoserAirPumpPin, LOW);
}

void print2digits(int number) {

  if (number >= 0 && number < 10) {
    lcd->write('0');
  }
  lcd->print(number);
}

void showTimeVals() {
  lcd->setCursor(0, 0);
  print2digits(t.hour);
  lcd->print(":");
  print2digits(t.min);
  lcd->print(":");
  print2digits(t.sec);
  lcd->print("  V="); lcd->print(getBatteryVoltage());
  lcd->setCursor(0, 1);
  lcd->print(F("Volume: ")); lcd->print(volume); lcd->print(" ml");
  lcd->setCursor(0, 2); lcd->print(F("Start time:")); print2digits(startTime.hour); lcd->print(":"); print2digits(startTime.min);
  lcd->setCursor(0, 3); lcd->print(F("Times a day: ")); lcd->print(timesAday);
}

void do_the_dosing() {
  //int quantity = 0;
  lcd->clear();
  lcd->home();
  lcd->print(F("Diatomixer Mixing"));
  mixer_run();  
  delay(80000);
  vacuum_run();
  pump_run_reverse(10); //run backwards 10 revolutions
  vacuum_stop();
  mixer_stop();   
  delay(1000);
  lcd->clear();
  lcd->home();
  long workTimer = millis();
  //vacuum_run();
  delay(50);
  unsigned long doserevs = dose/MLPERROTATION;
  unsigned long rotations = doserevs +  IDLE_ROTATIONS;
  lcd->home();
  lcd->print(F("Dosing Diatomix"));
  lcd->setCursor(5, 1);
  lcd->print(dose);
  lcd->print("ml");
 
  pump_run(rotations);
  delay(1000);
  pump_stop();
  lcd->clear();
  lcd->home();
  lcd->print(F("Reverse Pumping"));
  vacuum_run();
  pump_run_reverse(15); //run backwards 15 revolutions
  
  pump_stop();
  delay(1000);
  lcd->clear();
  lcd->home();
  lcd->print(F("Running Air Pumps"));
  //vacuum_run();
  delay(pumpRunTime * 60000);
  vacuum_stop();
  delay(1000);
  shut_down_everything();
  lcd->clear();
}

void pump_stop() {
  digitalWrite(MotorEnPin, HIGH);
}

void pump_run(unsigned long revs) {
  digitalWrite(MotorEnPin, LOW);
  unsigned long steps = stepsPerRevolution * revs;
  motor.move(steps);
  motor.runToPosition();
}

void pump_run_reverse(unsigned long revs) {
  digitalWrite(MotorEnPin, LOW);
  unsigned long steps = -stepsPerRevolution * revs;
  motor.move(steps);
  motor.runToPosition();
}
