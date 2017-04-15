#include <SD.h>                     // SD card library
#include <SPI.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>      // for BMP085 altimeter
#include <Adafruit_LSM303_U.h>      // for LSM303 accelerometer

#include <RocketMath.h>             // for Kalman filter and trajectory equations
#include "Vehicle.h"                // for rocket vehicle characteristics

#define NOMINAL_DT                  0.05 // seconds
#define INDICATOR_PIN               3 // blinks with every tick
#define ERROR_PIN                   4 // turns on if a fatal error occurs

#define ON_LAUNCHPAD                0 // indicates rocket is on the launchpad. accel = 0.
#define MOTOR_BURN                  1 // indicates motor is burning. accel > 0.
#define APOGEE_COAST                2 // indicates motor has burned out. accel < 0.

int stage = ON_LAUNCHPAD;           // current state of the vehicle

Adafruit_LSM303_Accel_Unified       lsm = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_BMP085_Unified             bmp = Adafruit_BMP085_Unified(10085);

KalmanFilter filter(0,0.0001);
File sensorData;

using namespace Equation;

void setup()
{
    // set the pinmode for the diagnostic LEDs
    pinMode(INDICATOR_PIN, OUTPUT);
    pinMode(ERROR_PIN, OUTPUT);

    // flash the LEDs to ensure they're all working
    digitalWrite(INDICATOR_PIN, HIGH);
    digitalWrite(ERROR_PIN, HIGH);
    delay(1000);
    digitalWrite(INDICATOR_PIN, LOW);
    digitalWrite(ERROR_PIN, LOW);

    // initialize the LSM303, the BMP085, and the SD card
    if(!lsm.begin() || !bmp.begin() || !SD.begin())
    {
        fatal_error(1);
    }

    // initialize the log file
    char filename[] = "LOG000.csv";
    for (uint8_t i = 0; i < 1000; i++)
    {
        filename[3] = i/100 + '0';
        filename[4] = (i/10)%10 + '0';
        filename[5] = i%10 + '0';

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

    // initialize the Kalman filter
    // state transition matrix, F
    filter.F[0][0] = 1;
    filter.F[1][1] = 1;
    // sensor covariance matrix, R
    filter.R[0][0] = 0.08;
    filter.R[1][1] = 0.02;
    // state matrix, X
    filter.X[0][0] = 0;
    filter.X[1][0] = 0;
    // uncertainty matrix, P
    filter.P[0][0] = 100;
    filter.P[1][1] = 100;

    sensorData.println(filename);
    sensorData.println("time,alt,accel,vel");
    delay(NOMINAL_DT * 1000);
}

void loop()
{
    update_indicator();
    static double alt_prev;
    double current_time, dt, altitude, accel;
    update_values(&current_time, &dt, &altitude, &accel);

    // filter wizardry to clean up alt and accel data
    filter.F[0][1] = 0.2 * dt;
    float Z[2] = {altitude, accel};
    float* X = filter.step((float*) Z);
    altitude = X[0];
    accel = X[1];
    double velocity = (altitude - alt_prev)/dt;

    // stage-specific progression logic
    if (stage == ON_LAUNCHPAD && velocity > 20)
    {
        stage++;
    }
    if (stage == MOTOR_BURN)
    {
        static int low_accel_count;
        if (accel < -10) low_accel_count++;
        if (low_accel_count > 15) stage++;
    }
    if (stage == APOGEE_COAST && FLIGHT_NUMBER)
    {
        /*
         * alt_next and vel_next are the predicted state of the rocket in
         * dt seconds, assuming the flaps are deployed, and are found by
         * solving the following differential equation:
         *
         *     dv/dt + (kv^2 / m) + g = 0
         *
         * This yields the model found in Equations.h.
         * ta_predict is the time to apogee which is predicted for the
         * next step, and apo_predict is the apogee altitude,
         * assuming the flaps are retracted for the rest of the flight.
         * As long as this predicted altitude is greater than the target,
         * it is desirable to lower it for this timestep, so the flaps
         * are opened. Otherwise, opening the flaps this step will cause the
         * vehicle to brake too hard, so the flaps are closed.
         */

        const int vmin = 8; // minimum velocity for flap control

        // predictive calculations determine where vehicle will be next step
        double alt_next = alt(altitude, velocity, DRY_MASS, K_ACTIVE, dt);
        double vel_next = vel(velocity, DRY_MASS, K_ACTIVE, dt);

        // describe the trajectory the vehicle will be on during next step
        double ta_predict = t_a(vel_next, DRY_MASS, K_PASSIVE);
        double apo_predict = alt(alt_next, vel_next, DRY_MASS, K_PASSIVE, ta_predict);

        // if the predicted altitude is acceptable, engage the flaps
        if (apo_predict > TARGET_ALT && vel_next > vmin)
        {
            // TODO: flaps ON
        }
        else // otherwise, retract the flaps
        {
            // TODO: flaps OFF
        }

        // permanently disable flaps if velocity is low enough
        if (velocity < vmin)
        {
            // TODO: flaps OFF
            stage++;
        }
    }

    alt_prev = altitude; // save previous altitude for faux derivative

    // write to log file
    sensorData.print(current_time, 4);
    sensorData.print(",");
    sensorData.print(altitude);
    sensorData.print(",");
    sensorData.print(accel);
    sensorData.print(",");
    sensorData.println(velocity);
    sensorData.flush();

    // wait for next tick if computations were fast enough
    double compTime = (double) millis()/1000 - current_time;
    if (compTime < NOMINAL_DT)
    {
        delay((NOMINAL_DT - compTime)*1000);
    }
}

void update_values(double* current_time, double* dt, double* alt, double* accel)
{
    // get current time and time since last call
    static double lastTime;
    *current_time = (double) millis()/1000;
    *dt = *current_time - lastTime;
    lastTime = *current_time;

    // collect acceleration and altitude measurements
    sensors_event_t alt_event;        // altitude event from BMP085
    sensors_event_t accel_event;      // acceleration event from LSM303
    bmp.getEvent(&alt_event);
    *alt = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA,
        alt_event.pressure) - ALT_BIAS;
    lsm.getEvent(&accel_event);
    *accel = accel_event.acceleration.z - ACCEL_BIAS;
}

void update_indicator()
{
    static bool indicator;
    if (indicator)
    {
        analogWrite(INDICATOR_PIN, 50);
    }
    else
    {
        analogWrite(INDICATOR_PIN, 0);
    }
    indicator = !indicator;
}

void fatal_error(int error)
{
    while(1)
    {
        for (int i = 0; i < error; i++)
        {
            digitalWrite(ERROR_PIN, HIGH);
            delay(150);
            digitalWrite(ERROR_PIN, LOW);
            delay(150);
        }
        delay(500);
    }
}
