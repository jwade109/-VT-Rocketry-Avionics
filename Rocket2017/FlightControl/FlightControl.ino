#include <SD.h>
#include <SPI.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_LSM303_U.h>

#include <RocketMath.h>             // for Kalman filter and trajectory equations
#include "Vehicle.h"                // for rocket vehicle characteristics

Adafruit_LSM303_Accel_Unified       lsm = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_BMP085_Unified             bmp = Adafruit_BMP085_Unified(10085);

KalmanFilter filter(0.5, 0.01);

double previous_time, current_time;

void setup()
{
    Serial.begin(9600);

    if(!lsm.begin())
    {
        Serial.println("No LSM303 detected: check wiring");
        while(1);
    }
  
    if(!bmp.begin())
    {
        Serial.print("No BMP085 detected: check wiring or I2C ADDR");
        while(1);
    }

    if (!SD.begin())
    {
        Serial.println("SD card initialization failed");
    }
    
    File sensorData = SD.open("RocketData.csv", FILE_WRITE);
  
    if(sensorData)
    {
        sensorData.print("Time(ms)");
        sensorData.print("\t");
        sensorData.print("Altitude(m)");
        sensorData.print("\t");
        sensorData.print("AccelX(m/s^2)");
        sensorData.print("\t");
        sensorData.print("AccelY(m/s^2)");
        sensorData.print("\t");
        sensorData.print("AccelZ(m/s^2)");
        sensorData.print("\t");
        sensorData.close();
    }

    previous_time = (double) millis()/1000;
    delay(NOMINAL_DT * 1000);
    current_time = (double) millis()/1000;
    delay(NOMINAL_DT * 1000);
    Serial.println("\n");
}

void update_time(double* current_time, double* dt)
{
    static double lastTime;
    *current_time = (double) millis()/1000;
    *dt = *current_time - lastTime;
    lastTime = *current_time;
}

void loop()
{
    double current_time, dt;
    update_time(&current_time, &dt);
    
    sensors_event_t alt_event;        // altitude event from BMP085
    sensors_event_t accel_event;      // acceleration event from LSM303
    
    float altitude;
    float accelX;
    float accelY;
    float accelZ;
    
    if(bmp.getEvent(&alt_event))
    {
        altitude = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, alt_event.pressure);
    }
    
    if(lsm.getEvent(&accel_event))
    {
        accelX = accel_event.acceleration.x;
        accelY = accel_event.acceleration.y;
        accelZ = accel_event.acceleration.z;
    }

    String dataString = String(current_time, 3) + "     " +
                        String(1/dt)            + "     " +
                        String(altitude)        + "     " +
                        String(accelX)          + "     " +
                        String(accelY)          + "     " +
                        String(accelZ);

    int vel = 0;
    
    /* 
     * Example syntax for trajectory equations
     */
    double ta = Equation::t_a(vel, DRY_MASS, K_ACTIVE);
    double alt = Equation::alt(alt, vel, DRY_MASS, K_ACTIVE, ta);
    
    /* 
     * Example syntax for Kalman filter
     */
    float Z[MEAS] = {alt, vel};
    float* X = filter.step((float*) Z);
    
    File sensorData = SD.open("RocketData.csv", FILE_WRITE);
    
    Serial.println(dataString);
    
    if(sensorData)
    {
        sensorData.print(dataString);
        sensorData.close();
    }

    double compTime = (double) millis()/1000 - current_time;

    if (compTime < NOMINAL_DT)
    {
        delay((NOMINAL_DT - compTime)*1000);
    }
}
