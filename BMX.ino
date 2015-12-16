//---------------------
//SD
/*
* SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4
 *
*/
#include <SPI.h>
#include <SD.h>

#define chipSelect 4

#define SERIAL_LOG
#define SD_LOG

//---------------------
//IMU

#include "NAxisMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>

NAxisMotion mySensor;                 //Object that for the sensor
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 40;          //To stream at 25Hz without using additional timers (time period(ms) =1000/frequency(Hz))
bool updateSensorData = true;         //Flag to update the sensor data. Default is true to perform the first read before the first stream

void setup() //This code is executed once
{
  
  #ifdef SERIAL_LOG
  Serial.begin(9600);           //Initialize the Serial Port to view information on the Serial Monitor
  #endif //SERIAL_LOG

  //init IMU
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor. 
  //Sensor Initialization
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);  //The default is AUTO. Changing to manual requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires lesser reads to the sensor
  mySensor.updateAccelConfig();
  updateSensorData = true;

  #ifdef SERIAL_LOG
  Serial.println();
  Serial.println("Default accelerometer configuration settings...");
  Serial.print("Range: ");
  Serial.println(mySensor.readAccelRange());
  Serial.print("Bandwidth: ");
  Serial.println(mySensor.readAccelBandwidth());
  Serial.print("Power Mode: ");
  Serial.println(mySensor.readAccelPowerMode());
 
  Serial.print("Initializing SD card...");
  #endif //SERIAL_LOG

  #ifdef SD_LOG
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    #ifdef SERIAL_LOG
    Serial.println("Card failed, or not present");
    #endif //SERIAL_LOG
    // don't do anything more:
    while(1);
  }
  #ifdef SERIAL_LOG
  Serial.println("card initialized.");
  Serial.println("initialize log file to SD card...");
  #endif //SERIAL_LOG

  String csvHeader = "Timestamp[ms],AccX[m/s2],AccY[m/s2],AccZ[m/s2]";
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(csvHeader);
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    #ifdef SERIAL_LOG
    Serial.println("error opening datalog.txt");
    #endif //SERIAL_LOG
  }
  #endif //SD_LOG
}

void loop() //This code is looped forever
{
  if (updateSensorData)  //Keep the updating of data as a separate task
  {
    mySensor.updateAccel();        //Update the Accelerometer data
    mySensor.updateLinearAccel();  //Update the Linear Acceleration data
    mySensor.updateGravAccel();    //Update the Gravity Acceleration data
    mySensor.updateCalibStatus();  //Update the Calibration Status
    updateSensorData = false;
  }
  if ((millis() - lastStreamTime) >= streamPeriod)
  {
    lastStreamTime = millis();

    float Ax = mySensor.readAccelX();
    float Ay = mySensor.readAccelY();
    float Az = mySensor.readAccelZ();

    #ifdef SERIAL_LOG
    Serial.print("Time: ");
    Serial.print(lastStreamTime);
    Serial.print("ms ");

    Serial.print("      aX: ");
    Serial.print(Ax,8); //Accelerometer X-Axis data
    Serial.print("m/s2 ");

    Serial.print(" aY: ");
    Serial.print(Ay,8);  //Accelerometer Y-Axis data
    Serial.print("m/s2 ");

    Serial.print(" aZ: ");
    Serial.print(Az,8);  //Accelerometer Z-Axis data
    Serial.print("m/s2 ");

    Serial.print("      lX: ");
    Serial.print(mySensor.readLinearAccelX(),8); //Linear Acceleration X-Axis data
    Serial.print("m/s2 ");

    Serial.print(" lY: ");
    Serial.print(mySensor.readLinearAccelY(),8);  //Linear Acceleration Y-Axis data
    Serial.print("m/s2 ");

    Serial.print(" lZ: ");
    Serial.print(mySensor.readLinearAccelZ(),8);  //Linear Acceleration Z-Axis data
    Serial.print("m/s2 ");

    Serial.print("      gX: ");
    Serial.print(mySensor.readGravAccelX(),8); //Gravity Acceleration X-Axis data
    Serial.print("m/s2 ");

    Serial.print(" gY: ");
    Serial.print(mySensor.readGravAccelY(),8);  //Gravity Acceleration Y-Axis data
    Serial.print("m/s2 ");

    Serial.print(" gZ: ");
    Serial.print(mySensor.readGravAccelZ(),8);  //Gravity Acceleration Z-Axis data
    Serial.print("m/s2 ");

    Serial.print("      C: ");
    Serial.print(mySensor.readAccelCalibStatus());  //Accelerometer Calibration Status (0 - 3)

    Serial.println();
    #endif //SERIAL_LOG

    //log data to SD card in csv
    String dataString = "";
    dataString += String(lastStreamTime);
    dataString += ",";
    dataString += String(Ax);
    dataString += ",";
    dataString += String(Ay);
    dataString += ",";
    dataString += String(Az);

    // open the file. note that only one file can be open at a time,
    #ifdef SD_LOG
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
     // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
    }
    // if the file isn't open, pop up an error:
    else {
      #ifdef SERIAL_LOG
      Serial.println("error opening datalog.txt");
      #endif //SERIAL_LOG     
    }
    #endif //SD_LOG

    updateSensorData = true;
  }
}
