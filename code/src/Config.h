//comment out the line below to not send debug messages on the serial port.
#define DEBUG

//Soil temperture set point
#define SOIL_SETPOINT 25.0
//Soil maximum temp set point
#define SOIL_MAX_TEMP 30.0
//How long do we wait to throw an alarm if soil temp goes over the max temp (in milliseconds)
#define SOIL_HEATER_OVERTEMP_ALARM_DELAY 30000

//PWM setup
#define PWM_HEATER_CHANNEL 0
#define PWM_HEATER_FREQ 1000

//PID parameters
//the bigger the number the harder the controller pushes
#define PID_P 50.0
//the SMALLER the number (except for 0, which turns it off) the more quickly the controller reacts to load changes, but the greater the risk of oscillations
#define PID_I 0.2
//the bigger the number the more the controller dampens oscillations (to the point where performance can be hindered)
#define PID_D 0.5