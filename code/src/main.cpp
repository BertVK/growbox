#include <Arduino.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <SHT2x.h>
#include <HardwareSetup.h>
#include <Config.h>

OneWire oneWire(PIN_ONE_WIRE);
DallasTemperature soiltempsensor(&oneWire);
SHT2x airSensor;

float soilTemperature, soilSetpoint, soilMaxTemp, airHumity, airTemperature;
double Setpoint, Input, Output;
unsigned long soilHeaterOverTempTime;
unsigned long soilHeaterOvertempAlarmDelay;
byte soilHeaterOverTempAlarm;

enum switchStates
{
  OFF,
  ON
};
enum results
{
  NO,
  YES
};
enum machineStates
{
  HOLD,
  RUN,
  ALARM
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

void updateDisplay()
{
#ifdef DEBUG
  //Air paramters
  Serial.print("Air  -> temp : ");
  Serial.print(airTemperature, 1);
  Serial.print(" - humidity : ");
  Serial.print(airHumity, 0);
  Serial.println("%");

//Soil parameters
  Serial.print("Soil -> temp : ");
  Serial.print(soilTemperature, 1);
  Serial.print(" - Setpoint : ");
  Serial.print(soilSetpoint, 1);
  Serial.print(" - PID Output : ");
  Serial.print(myPID.GetMode() ? "A" : "M");
  Serial.print(" / ");
  float power = (Output / 255.0) * 100.0;
  Serial.print(power, 0);
  Serial.print("% - Alarm = ");
  Serial.println(soilHeaterOverTempAlarm);
  Serial.println("");
#endif
}

void initSoilTempSensor()
{
  soiltempsensor.begin();
  soiltempsensor.setResolution(12);
}

void initAirSensor()
{
  airSensor.begin(PIN_I2C_SDA, PIN_I2C_SCL);
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

float getAirHumidity()
{
  if (airSensor.isConnected())
  {
    bool b = airSensor.read();
    float humidity = airSensor.getHumidity();
    return humidity;
  }
  return -1.0;
}

float getAirTemperature()
{
  if (airSensor.isConnected())
  {
    bool b = airSensor.read();
    float temp = airSensor.getTemperature();
    return temp;
  }
  return -1.0;
}

void setState(uint8_t state)
{
  switch (state)
  {
  case HOLD:
    // showAlarms();
    myPID.SetMode(MANUAL);
    Setpoint = 0;
    Output = 0;
  case ALARM:
    // showAlarms();
    myPID.SetMode(MANUAL);
    Setpoint = 0;
    Output = 0;
    switchAlarmLED(ON);
  default:
    break;
  }
}

float getSoilTemp()
{
  float celsius;
  soiltempsensor.requestTemperaturesByIndex(0);
  celsius = soiltempsensor.getTempCByIndex(0);
  // Dirty hack for when temp sensor goes bad.
  while (celsius == 85.0)
  {
    initSoilTempSensor();
    soiltempsensor.requestTemperaturesByIndex(0);
    celsius = soiltempsensor.getTempCByIndex(0);
  }
  return celsius;
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
}

void setSoilHeater(uint8_t setpoint)
{
  ledcWrite(PWM_HEATER_CHANNEL, setpoint);
}

void setupIO()
{
  // Set the IO
  pinMode(PIN_ONE_WIRE, INPUT);
  pinMode(PIN_LED_ALARM, OUTPUT);
  pinMode(PIN_HEATER_FET, OUTPUT);

  // Setup PWM
  // For the heater we always use 8bit as this resolution is equal to the PID output resolution
  uint32_t freq = ledcSetup(PWM_HEATER_CHANNEL, PWM_HEATER_FREQ, 8);
#ifdef DEBUG
  Serial.print("Setting up PWM for heater. Result = ");
  Serial.println(freq);
#endif
  // Connect PWM channel 0 to heater output pin
  ledcAttachPin(PIN_HEATER_FET, PWM_HEATER_CHANNEL);

  // Set startup values
  // Switch heater off
  ledcWrite(PWM_HEATER_CHANNEL, 0);
  // Switch alarm LED off
  digitalWrite(PIN_LED_ALARM, LOW);
}

void setupParams()
{
  // initialize the variables
  // Soil set point
  soilSetpoint = SOIL_SETPOINT;
  // what is the absolute max temp to alow from the water sensor
  soilMaxTemp = SOIL_MAX_TEMP;
  // How long may the water temp be higher than soilMaxTemp before the alarm is triggered
  soilHeaterOvertempAlarmDelay = SOIL_HEATER_OVERTEMP_ALARM_DELAY;
  soilTemperature = 0;
  soilHeaterOverTempAlarm = OFF;
}

void setup(void)
{
#ifdef DEBUG
  Serial.begin(115200);
#endif
  setupIO();
  setupParams();
  initSoilTempSensor();
  Setpoint = soilSetpoint;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  updateDisplay();
  switchAlarmLED(OFF);

  //Init air sensor
  initAirSensor();

  // Let everything settle
  delay(500);
}

void loop(void)
{
  soilTemperature = getSoilTemp();
  checkAlarmStates();
  if (soilHeaterOverTempAlarm == ON)
  {
    setState(ALARM);
  }
  else
  {
    runPID();
    setSoilHeater(Output);
    airHumity = getAirHumidity();
    airTemperature = getAirTemperature();
  }
  updateDisplay();
}