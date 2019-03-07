/*
 * Project AssetTrackerDevice
 * Description:
 * Author: pbutler@basdesign.com
 * Date: 2019-02-27
 */

#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055_Photon.h>
#include <Adafruit_MCP9808.h>
#include <Adafruit_SSD1306.h>

#include "Particle.h"

SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);

Adafruit_BNO055 bno = Adafruit_BNO055(55);
#define BNO055_SAMPLERATE_DELAY_MS (1000)

Adafruit_MCP9808 mcp = Adafruit_MCP9808();

#define OLED_RESET D4
Adafruit_SSD1306 display(OLED_RESET);

// what's the name of the hardware serial port?
#define GPSSerial Serial1

// What ports are we using for GPIO input?
int measurement1Pin = A0;
int measurement2Pin = A1;

// Calibrations for measuting voltage on the measurement pins
float pin1Cal = 249.965;
float pin2Cal = 250;

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

// Set whether you want the device to publish data to the internet by default here.
// 1 will Particle.publish AND Serial.print, 0 will just Serial.print
// Extremely useful for saving data while developing close enough to have a cable plugged in.
// You can also change this remotely using the Particle.function "tmode" defined in setup()
int transmittingData = 1;

// Used to manage screen updates
long lastScreenUpdate = 0;
float intervalScreenupdate = 2000;

// Used to keep track of the last time we published data
long lastPublish = 0;

// How long between publishes if the tracker isn't moving
float intervalMinutesNotMoving = 15;

// How long between publishes if the tracker is moving
float intervalMinutesMoving = 0.5;

// Speed threshold to switch between intervalMinutesNotMoving and intervalMinutesMoving
float movingThreshold = 0.75;

// Allows you to remotely change whether a device is publishing to the cloud
// or is only reporting data over Serial. Saves data when using only Serial!
// Change the default at the top of the code.
int transmitMode(String command)
{
  transmittingData = atoi(command);
  return 1;
}

void displayTelemetry(void)
{
  // Reset the timer
  lastScreenUpdate = millis();
  // display.clearDisplay();
  // display.setCursor(0, 0);
  // display.println("T1: " + String(mcp.readTempC(), 2));
  // display.display();
}

// Actively ask for a full fix reading if you're impatient.
int publishFix(String command)
{

  // Reset the lastPublish counter
  lastPublish = millis();

  // Get a new sensor event
  sensors_event_t event;
  bno.getEvent(&event);

  String pubStr = String(GPS.lat) + String(',') + String(GPS.latitude) + String(',') + String(GPS.lon) + String(',') + String(GPS.longitude) + String(',') + String(GPS.speed, 1) + String(',') + String(GPS.angle, 0) + String(',') + String(GPS.fixquality) + String(',') + String(event.orientation.x, 0) + String(',') + String(event.orientation.y, 0) + String(',') + String(event.orientation.z, 0) + String(',') + String(mcp.readTempC(), 2) + String(',') + String(analogRead(measurement1Pin) / pin1Cal, 2) + String(',') + String(analogRead(measurement2Pin) / pin2Cal, 2);

  Serial.println(pubStr);

  Particle.publish("fix", pubStr, 60, PRIVATE);

  return 1;
}

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
  Serial.print("Accelerometer: ");
  Serial.print(calibData.accel_offset_x);
  Serial.print(" ");
  Serial.print(calibData.accel_offset_y);
  Serial.print(" ");
  Serial.print(calibData.accel_offset_z);
  Serial.print(" ");

  Serial.print("\nGyro: ");
  Serial.print(calibData.gyro_offset_x);
  Serial.print(" ");
  Serial.print(calibData.gyro_offset_y);
  Serial.print(" ");
  Serial.print(calibData.gyro_offset_z);
  Serial.print(" ");

  Serial.print("\nMag: ");
  Serial.print(calibData.mag_offset_x);
  Serial.print(" ");
  Serial.print(calibData.mag_offset_y);
  Serial.print(" ");
  Serial.print(calibData.mag_offset_z);
  Serial.print(" ");

  Serial.print("\nAccel Radius: ");
  Serial.print(calibData.accel_radius);

  Serial.print("\nMag Radius: ");
  Serial.print(calibData.mag_radius);
}

// setup() runs once, when the device is first turned on.
void setup()
{

  // These functions exposed via the cloud API
  Particle.function("tmode", transmitMode);
  Particle.function("fix", publishFix);

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // MCP9808 sensor setup
  while (!mcp.begin())
  {
    Serial.println("Ooops, no MCP9808 detected ... Check your wiring or I2C ADDR");
    delay(1000);
  }

  /* BNO55 sensor setup */
  Serial.println("Orientation Sensor Test");
  Serial.println("");
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
  bno.getSensor(&sensor);

  if (bnoID != sensor.sensor_id)
  {
    Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
  }
  else
  {
    Serial.println("\nFound calibration in EEPROM, loading into BNO55.");
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);
    bno.setSensorOffsets(calibrationData);
    foundCalib = true;
    delay(1000);
  }

  //Crystal must be configured AFTER loading calibration data into BNO055.
  bno.setExtCrystalUse(true);

  sensors_event_t event;
  bno.getEvent(&event);
  if (!foundCalib)
  {
    Serial.println("Please Calibrate Sensor: ");
    while (!bno.isFullyCalibrated())
    {
      bno.getEvent(&event);

      Serial.print("X: ");
      Serial.print(event.orientation.x, 4);
      Serial.print("\tY: ");
      Serial.print(event.orientation.y, 4);
      Serial.print("\tZ: ");
      Serial.print(event.orientation.z, 4);

      /* Optional: Display calibration status */
      displayCalStatus();

      /* New line for the next sample */
      Serial.println("");

      /* Wait the specified delay before requesting new data */
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
  }

  Serial.println("--------------------------------");
  Serial.println("Fully Calibrated: ");
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);
  displaySensorOffsets(newCalib);

  Serial.println("\n\nStoring calibration data to EEPROM...");

  eeAddress = 0;
  bno.getSensor(&sensor);
  bnoID = sensor.sensor_id;

  EEPROM.put(eeAddress, bnoID);

  eeAddress += sizeof(long);
  EEPROM.put(eeAddress, newCalib);
  Serial.println("Data stored to EEPROM.");
  Serial.println("\n--------------------------------\n");
  delay(500);
}

// loop() runs over and over again, as quickly as it can execute.
void loop()
{

  // Read data from the GPS in the 'main loop'
  char c = GPS.read();

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived())
  {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return;                       // we can fail to parse a sentence in which case we should just wait for another
  }

  // Display update
  if (millis() - lastScreenUpdate > intervalScreenupdate)
  {
    displayTelemetry();
  }

  if (GPS.fix)
  {
    // If we're moving, publish at intervalMinutesMoving
    // otherwise publish at intervalMinutesNotMoving
    if (GPS.speed >= movingThreshold)
    {
      if (millis() - lastPublish > intervalMinutesMoving * 60 * 1000)
      {
        publishFix("update");
      }
    }
    else if (millis() - lastPublish > intervalMinutesNotMoving * 60 * 1000)
    {
      publishFix("update");
    }

    if (millis())

      // Hardware reset at midnight local
      if (GPS.hour == 8 && GPS.minute == 0 && GPS.seconds == 0)
      {
        System.reset();
      }
  }
}
