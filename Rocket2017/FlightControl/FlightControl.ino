#include <SD.h>
#include <SPI.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>

#include <RocketMath.h>

const int CS_pin = 4;
double previous_time;
double current_time;

Adafruit_9DOF                       dof = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified       accelerometer = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_BMP085_Unified             bmp = Adafruit_BMP085_Unified(10085);

void setup(){

    Serial.begin(9600);
    
    if(!accelerometer.begin())
    {
        Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
        while(1);
    }
  
    if(!bmp.begin())
    {
        Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }

    pinMode(CS_pin, OUTPUT);

    if (!SD.begin(CS_pin))
    {
        Serial.println("Card init. failed!"); /*initialize SD card dreader*/
    }
    
    File mySensorData = SD.open("RocketData.csv", FILE_WRITE);
  
    if(mySensorData)
    {
        mySensorData.print("Time(ms)");
        mySensorData.print("\t");
        mySensorData.print("Altitude(m)");
        mySensorData.print("\t");
        mySensorData.print("AccelX(m/s^2)");
        mySensorData.print("\t");
        mySensorData.print("AccelY(m/s^2)");
        mySensorData.print("\t");
        mySensorData.print("AccelZ(m/s^2)");
        mySensorData.print("\t");
        mySensorData.close();
    }

    current_time = millis()/1000;
}

void loop()
{
    previous_time = current_time;
    double dt = current_time - previous_time;
    current_time = millis()/1000;
    
    sensors_event_t event;            // Object for pressure and altitude - FOR BMP085
    sensors_event_t accel_event;      // Object for accelerometer - for accelation - 9DOF
    
    float altitude;
    float accelX;
    float accelY;
    float accelZ;
    
    if(bmp.getEvent(&event))
    {
        altitude = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, event.pressure);
    }
    else
    {
        Serial.print("Failed to get event");
    }
    
    if(accelerometer.getEvent(&accel_event))
    {
        accelX = accel_event.acceleration.x;
        accelY = accel_event.acceleration.y;
        accelZ = accel_event.acceleration.z;
    }
    
    String dataString = String(current_time * 1000) + "    " +
                        String(altitude) + "     " +
                        String(accelX) + "     " +
                        String(accelY) + "     " +
                        String(accelZ);
    
    File mySensorData = SD.open("RocketData.csv", FILE_WRITE);
    
    if(mySensorData)
    {
        mySensorData.print(dataString);
        mySensorData.close();
    }
    
    delay(20);
}
