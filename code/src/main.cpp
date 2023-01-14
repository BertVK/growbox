#include <Arduino.h>
#include <OneWire.h>
#include <DS18B20.h>
#include <PID_v1.h>
#include <ShiftRegister74HC595.h>
#include <LiquidCrystal.h>
#include <HardwareSetup.h>
#include <Config.h>

OneWire oneWire(PIN_ONE_WIRE);
DS18B20 tempSensor(&oneWire);
double soilTemperature, soilSetpoint;
unsigned int heaterSpeed;
unsigned int heaterTimeGranularity;
float soilMaxTemp;
float soilHeaterFailurePreviousTemp;
double Setpoint, Input, Output;
unsigned long lastTime, newTime;
unsigned long soilHeaterOverTempTime;
unsigned long soilHeaterOvertempAlarmDelay;
unsigned long soilHeaterFailureTime;
unsigned long soilHeaterFailureDelay;
byte heaterState, previousHeaterState, soilHeaterOverTempAlarm, soilHeaterFailureAlarm, result;

// create a global shift register object
// parameters: <number of shift registers> (data pin, clock pin, latch pin)
ShiftRegister74HC595<1> relays(PIN_RELAYS_DATA, PIN_RELAYS_CLOCK, PIN_RELAYS_LATCH);

LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

enum states
{
  OFF,
  ON
};
enum results
{
  NO,
  YES
};

// Specify the links and initial tuning parameters
double Kp = PID_P, Ki = PID_I, Kd = PID_D;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void runPID()
{
  // set INPUT to temp
  Input = soilTemperature;
  myPID.Compute();
}

void setupDisplay()
{
  // set up the LCD's number of columns and rows:
  lcd.begin(20, 4);
  lcd.clear();
}

void updateDisplay()
{
#ifdef DEBUG
  Serial.print("Temp : ");
  Serial.print(soilTemperature);
  Serial.print(" - Setpoint : ");
  Serial.print(soilSetpoint);
  Serial.print(" - PID Output : ");
  Serial.print(myPID.GetMode() ? "A" : "M");
  Serial.print(" / ");
  Serial.print(Output);
  Serial.print(" - Heater result : ");
  Serial.print(result);
  Serial.print(" / ");
  Serial.print(HEATER_TIME_GRANULARITY);
  Serial.print(" - Alarms = ");
  Serial.print(soilHeaterOverTempAlarm);
  Serial.print(" - ");
  Serial.println(soilHeaterFailureAlarm);
#endif
  if (soilHeaterFailureAlarm == ON)
  {
    lcd.setCursor(0, 0);
    lcd.print("SOIL HEATER FAILED");
  }
  else if (soilHeaterOverTempAlarm == ON)
  {
    lcd.setCursor(0, 0);
    lcd.print("SOIL HEATER OVERTEMP");
  }
  else
  {
    lcd.setCursor(1, 0);
    lcd.print(soilTemperature);
    lcd.print("°C");
  }
}

void getTemperatures()
{
  tempSensor.requestTemperatures();
  while (!tempSensor.isConversionComplete())
    ;
  soilTemperature = tempSensor.getTempC();
}

void setSoilHeater()
{
  if (previousHeaterState == OFF && heaterState == ON)
  {
#ifdef DEBUG
    Serial.println("HEATER IS ON");
#endif
    relays.set(RELAY_HEATER, LOW);
    previousHeaterState = ON;
  }
  if (previousHeaterState == ON && heaterState == OFF)
  {
#ifdef DEBUG
    Serial.println("HEATER IS OFF");
#endif
    relays.set(RELAY_HEATER, HIGH);
    previousHeaterState = OFF;
  }
}

void setSoilHeaterState()
{
  // Make sure the substrate does not rise above the maximum temperature
  if (soilTemperature > soilMaxTemp)
  {
    heaterState = OFF;
  }
  else
  {
    result = map(Output, 0, 255, 0, heaterTimeGranularity);
    if (result == 0)
    {
      heaterState = OFF;
    }
    else if (result == heaterTimeGranularity)
    {
      heaterState = ON;
    }
    else if (millis() < (lastTime + (result * heaterSpeed)))
    {
      heaterState = ON;
    }
    else
    {
      heaterState = OFF;
    }
  }
}

void checkAlarmStates()
{
  // Check if the soil temperature is not higher than the max temp for the delay time
  if (soilTemperature > soilMaxTemp)
  {
#ifdef DEBUG
    Serial.print("Soil max temp reached! Soil temp = ");
    Serial.println(soilTemperature);
#endif
    if (soilHeaterOverTempTime > 0)
    {
      if ((millis() - soilHeaterOverTempTime) > soilHeaterOvertempAlarmDelay)
      {
        soilHeaterOverTempAlarm = ON;
      }
    }
    else
    {
      soilHeaterOverTempTime = millis();
    }
  }
  else
  {
    soilHeaterOverTempTime = 0;
  }
  // Check if the water temp is not rising when the heater is on for a set time
  // Heater must be high enough to heat the water
  if (result > ((heaterTimeGranularity * HEATER_FAILURE_THRESHOLD) / 10))
  {
    // Did we store a previous time?
    if (soilHeaterFailureTime > 0)
    {
// Did the measure delay pass?
#ifdef DEBUG
      Serial.print("Soil heater failure time = ");
      Serial.print(millis() - soilHeaterFailureTime);
      Serial.print(" of ");
      Serial.println(soilHeaterFailureDelay);
#endif
      if ((millis() - soilHeaterFailureTime) > soilHeaterFailureDelay)
      {
// If the temperature did not rise during that delay, we have a problem
#ifdef DEBUG
        Serial.println("soilHeaterFailureAlarm IS SET TO ON");
#endif
        soilHeaterFailureAlarm = ON;
      }
    }
    // If the temp did rise, we reset the timer.
    if (soilHeaterFailurePreviousTemp < (soilTemperature - 0.1))
      soilHeaterFailureTime = millis();
    soilHeaterFailurePreviousTemp = soilTemperature;
  }
  else
  {
    soilHeaterFailureTime = 0;
  }
}

void switchAlarmLED(uint8_t state)
{
  if (state == ON)
  {
    digitalWrite(PIN_LED_ALARM, HIGH);
  }
  else
  {
    digitalWrite(PIN_LED_ALARM, LOW);
  }
}

void setupIO()
{
  // Set the IO
  pinMode(PIN_ONE_WIRE, INPUT);
  pinMode(PIN_LED_ALARM, OUTPUT);
  // LCD pinout
  pinMode(PIN_LCD_EN, OUTPUT);
  pinMode(PIN_LCD_RS, OUTPUT);
  pinMode(PIN_LCD_D4, OUTPUT);
  pinMode(PIN_LCD_D5, OUTPUT);
  pinMode(PIN_LCD_D6, OUTPUT);
  pinMode(PIN_LCD_D7, OUTPUT);

  // Set startup value
  digitalWrite(PIN_LED_ALARM, LOW);
}

void setupParams()
{
  // initialize the variables
  // Hot water set point
  soilSetpoint = SOIL_SETPOINT;
  // what is the absolute max temp to alow from the water sensor
  soilMaxTemp = SOIL_MAX_TEMP;
  // How long may the water temp be higher than soilMaxTemp before the alarm is triggered
  soilHeaterOvertempAlarmDelay = SOIL_HEATER_OVERTEMP_ALARM_DELAY;
  // The minimal length the heater can be on or off.
  heaterSpeed = HEATER_SPEED;
  heaterTimeGranularity = HEATER_TIME_GRANULARITY;

  soilHeaterFailureDelay = SOIL_HEATER_FAILURE_DELAY;

  soilTemperature = 0;

  heaterState = OFF;
  previousHeaterState = OFF;
  soilHeaterOverTempAlarm = OFF;
  soilHeaterFailureAlarm = OFF;
}

void setup()
{
  // Set up display
  setupDisplay();
  lcd.setCursor(0, 0);
  lcd.print("Starting Setup");
#ifdef DEBUG
  Serial.begin(115200);
#endif

  setupIO();
  setupParams();

  // Set all relays to HIGH, switching outputs OFF
  relays.setAllHigh();

  if (tempSensor.begin() == false)
  {
#ifdef DEBUG
    Serial.println("Error: Could not find soil temp sensor.");
#endif
    // STOP ALL PROCESSING
    while (1)
      ;
  }
  tempSensor.setResolution(12);
  tempSensor.requestTemperatures();
  while (!tempSensor.isConversionComplete())
    ;

  getTemperatures();

  // turn the PID on
  Setpoint = soilSetpoint;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);

  lastTime = millis();
  newTime = millis();
  updateDisplay();
  switchAlarmLED(OFF);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Setup done");
  delay(500);
}

void loop()
{
  getTemperatures();
  checkAlarmStates();
  if (soilHeaterOverTempAlarm == ON || soilHeaterFailureAlarm == ON)
  {
    // showAlarms();
    myPID.SetMode(MANUAL);
    Setpoint = 0;
    Output = 0;
    setSoilHeaterState();
    setSoilHeater();
    switchAlarmLED(ON);
    updateDisplay();
  }
  else
  {
    if (newTime < millis())
    {
      runPID();
      setSoilHeaterState();
      updateDisplay();
      lastTime = millis();
      newTime = lastTime + (heaterTimeGranularity * heaterSpeed);
    }
    else
    {
      setSoilHeaterState();
    }
    setSoilHeater();
  }
  updateDisplay();
}