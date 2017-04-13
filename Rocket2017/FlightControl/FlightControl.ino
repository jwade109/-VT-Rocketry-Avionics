#include <SD.h>                     // SD card library
#include <SPI.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>      // for BMP085 altimeter
#include <Adafruit_LSM303_U.h>      // for LSM303 accelerometer

#include <RocketMath.h>             // for Kalman filter and trajectory equations
#include "Vehicle.h"                // for rocket vehicle characteristics

#define NOMINAL_DT                  0.05 // seconds
#define INDICATOR_PIN               3 // blinks with every tick
#define STAGE_PIN                   4 // blinks according to stage
#define ERROR_PIN                   5 // turns on if a fatal error occurs

#define ON_LAUNCHPAD                0 // indicates rocket is on the launchpad. accel = 0.
#define MOTOR_BURN                  1 // indicates motor is burning. accel > 0.
#define APOGEE_COAST                2 // indicates motor has burned out. accel < 0.

int stage = ON_LAUNCHPAD;           // current state of the vehicle

Adafruit_LSM303_Accel_Unified       lsm = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_BMP085_Unified             bmp = Adafruit_BMP085_Unified(10085);

KalmanFilter filter(0.5, 0.01);
File sensorData;

void update_time(double* current_time, double* dt);
void update_indicator();
void fatal_error(int error);

void setup()
{
    Serial.begin(9600);

    // set the pinmode for the diagnostic LEDs
    pinMode(INDICATOR_PIN, OUTPUT);
    pinMode(STAGE_PIN, OUTPUT);
    pinMode(ERROR_PIN, OUTPUT);

    // initialize the LSM303, the BMP085, and the SD card
    if(!lsm.begin() || !bmp.begin() || !SD.begin())
    {
        fatal_error(1);
    }

    // initialize the log file
    char filename[] = "LOG00.csv";
    for (uint8_t i = 0; i < 100; i++)
    {
        filename[3] = i/10 + '0';
        filename[4] = i%10 + '0';

        if (!SD.exists(filename))
        {
            sensorData = SD.open(filename, FILE_WRITE);
            break;
        }
    }
    if (!sensorData)
    {
        fatal_error(2);
    }

    sensorData.println(filename);
    sensorData.println("time,alt,accel");
    delay(NOMINAL_DT * 1000);
}

void loop()
{
    update_indicator();

    double current_time, dt;
    update_time(&current_time, &dt);

    sensors_event_t alt_event;        // altitude event from BMP085
    sensors_event_t accel_event;      // acceleration event from LSM303

    float altitude = 0;
    float accel = 0;

    bmp.getEvent(&alt_event);
    altitude = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, alt_event.pressure);

    lsm.getEvent(&accel_event);
    accel = accel_event.acceleration.x; // acceleration along longitudinal axis

    if (FLIGHT_NUMBER) // this block only runs during the second flight
    {
        int vel = 100;
        double ta = Equation::t_a(vel, DRY_MASS, K_ACTIVE);
        double alt = Equation::alt(alt, vel, DRY_MASS, K_ACTIVE, ta);

        float Z[MEAS] = {alt, accel};
        float* X = filter.step((float*) Z);
    }

    sensorData.print(current_time, 4);
    sensorData.print(",");
    sensorData.print(altitude);
    sensorData.print(",");
    sensorData.println(accel);
    sensorData.flush();

    double compTime = (double) millis()/1000 - current_time;
    if (compTime < NOMINAL_DT)
    {
        delay((NOMINAL_DT - compTime)*1000);
    }
}

void update_time(double* current_time, double* dt)
{
    static double lastTime;
    *current_time = (double) millis()/1000;
    *dt = *current_time - lastTime;
    lastTime = *current_time;
}

void update_indicator()
{
    static bool indicator;
    if (indicator)
    {
        analogWrite(INDICATOR_PIN, 20);
    }
    else
    {
        analogWrite(INDICATOR_PIN, 0);
    }
    indicator = !indicator;
}

void fatal_error(int error)
{
    analogWrite(ERROR_PIN, 255);
    while(1);
}
