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

#include "Particle.h"

SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_MCP9808 mcp = Adafruit_MCP9808();

// what's the name of the hardware serial port?
#define GPSSerial Serial1

// What's the address of the display?
#define OLED_ADDR 0x3C

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

// setup() runs once, when the device is first turned on.
void setup()
{
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

  /* Initialize the BNO55 */
  while (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    delay(1000);
  }
  bno.setExtCrystalUse(true);

  // MCP9808 sensor setup
  while (!mcp.begin())
  {
    Serial.println("Ooops, no MCP9808 detected ... Check your wiring or I2C ADDR");
    delay(1000);
  }
  // mcp.setResolution(1);

  // // Set up the display
  // display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  // display.clearDisplay();
  // display.display();

  // // display a pixel in each corner of the screen
  // display.drawPixel(0, 0, WHITE);
  // display.drawPixel(127, 0, WHITE);
  // display.drawPixel(0, 63, WHITE);
  // display.drawPixel(127, 63, WHITE);

  // // display a line of text
  // display.setTextSize(1);
  // display.setTextColor(WHITE);
  // display.setCursor(27, 30);
  // display.print("Hello, world!");

  // // update display with all of the above graphics
  // display.display();

  // These functions exposed via the cloud API
  Particle.function("tmode", transmitMode);
  Particle.function("fix", publishFix);
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

  // If we're moving, publish at intervalMinutesMoving
  // otherwise publish at intervalMinutesNotMoving
  if (GPS.fix && GPS.speed >= movingThreshold)
  {
    if (millis() - lastPublish > intervalMinutesMoving * 60 * 1000)
    {
      publishFix("update");
      // Remember when we published
      lastPublish = millis();
    }
  }
  else if (GPS.fix && millis() - lastPublish > intervalMinutesNotMoving * 60 * 1000)
  {
    publishFix("update");
    // Remember when we published
    lastPublish = millis();
  }

  // Hardware reset at midnight
  if (GPS.hour == 8 && GPS.minute == 0 && GPS.seconds == 0)
  { // Midnight local
    System.reset();
  }
}