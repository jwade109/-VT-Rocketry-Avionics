#include <SD.h>
#include <SPI.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_LSM303_U.h>

#include <RocketMath.h>             // for Kalman filter and trajectory equations
#include "Vehicle.h"                // for rocket vehicle characteristics

#define NOMINAL_DT                  0.05 // seconds
#define INDICATOR_PIN               3

Adafruit_LSM303_Accel_Unified       lsm = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_BMP085_Unified             bmp = Adafruit_BMP085_Unified(10085);

KalmanFilter filter(0.5, 0.01);
File sensorData;

void setup()
{
    Serial.begin(9600);

    pinMode(INDICATOR_PIN, OUTPUT);
    
    if(!lsm.begin())
    {
        Serial.println("No LSM303 detected: check wiring");
        while(1);
    }
    Serial.println("LSM303 initialized");
    delay(200);
  
    if(!bmp.begin())
    {
        Serial.print("No BMP085 detected: check wiring or I2C ADDR");
        while(1);
    }
    Serial.println("BMP085 initialized");
    delay(200);

    if (!SD.begin())
    {
        Serial.println("SD card initialization failed");
        while(1);
    }
    Serial.println("SD card initialized");
    delay(200);
    
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
    
    if (sensorData)
    {
        Serial.print("Writing to ");
        Serial.println(filename);
        delay(300);
    }
    else
    {
        Serial.print("Could not initialize ");
        Serial.println(filename);
        while(1);
    }

    sensorData.println(filename);
    sensorData.println("time,alt,accel");
    delay(NOMINAL_DT * 1000);
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
