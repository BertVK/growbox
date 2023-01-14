//comment out the line below to not send debug messages on the serial port.
#define DEBUG

//The minimal length the heater can be on or off. Minimum value is 100 (0.1 sec)
#define HEATER_SPEED 2000
#define HEATER_TIME_GRANULARITY 20
//Threshold above which the heating is checked, expressed in 0 to 10. Above this value the temp sensor mush sense changes in the temp to work
#define HEATER_FAILURE_THRESHOLD 6
//Soil maximum temp set point
#define SOIL_MAX_TEMP 35.0
//Soil temperture set point
#define SOIL_SETPOINT 25.0
//How long do we wait to throw an alarm if soil temp goes over the max temp (in milliseconds)
#define SOIL_HEATER_OVERTEMP_ALARM_DELAY 10000
//Hpw long do we wait to throw an alarms if the temperture does not rise when the heater is on 6 or above (in milliseconds)
#define SOIL_HEATER_FAILURE_DELAY 30000

//define the ports of the relay array. Array is 0-based
#define RELAY_HEATER 0

//PID parameters
//the bigger the number the harder the controller pushes
#define PID_P 50.0
//the SMALLER the number (except for 0, which turns it off) the more quickly the controller reacts to load changes, but the greater the risk of oscillations
#define PID_I 0.2
//the bigger the number the more the controller dampens oscillations (to the point where performance can be hindered)
#define PID_D 0.5