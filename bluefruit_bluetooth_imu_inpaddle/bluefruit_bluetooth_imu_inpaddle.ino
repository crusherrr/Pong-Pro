//-----------------BLUETOOTH STUFF HERE----------------
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

//-----------------------IMU STUFF HERE-----------
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;
#define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

int forehandstat;
int backhandstat;

int state = 0;

//-----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1)
      ;
  }
//---------------SETUP FOR BLUETOOTH
  //while (!Serial);  // required for Flora & Micro
  //delay(500);

  //Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }

}

void loop() {
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }

//  if ((lastPrint + PRINT_SPEED) < millis())
//    {
//    printGyro();  // Print "G: gx, gy, gz"
//    printAccel(); // Print "A: ax, ay, az"
//    printMag();   // Print "M: mx, my, mz"
//    // Print the heading and orientation for fun!
//    // Call print attitude. The LSM9DS1's mag x and y
//    // axes are opposite to the accelerometer, so my, mx are
//    // substituted for each other.
//    printAttitude(imu.ax, imu.ay, imu.az,
//                 -imu.my, -imu.mx, imu.mz);
//    Serial.println();
//
//    lastPrint = millis(); // Update lastPrint time
//
//
//    }

  //testforehand();

  //testbackhand();
testmove();
  // Check for user input
  char inputs[BUFSIZE+1];

  if ( getUserInput(inputs, BUFSIZE) )
  {
    // Send characters to Bluefruit
    Serial.print("[Send] ");
    Serial.println(backhandstat);
    Serial.println(forehandstat);

    ble.print("AT+BLEUARTTX=");
    ble.println(backhandstat);
    ble.println(forehandstat);

    // check response stastus
    if (! ble.waitForOK() ) {
      Serial.println(F("Failed to send?"));
    }
  }

  // Check for incoming characters from Bluefruit
  ble.println("AT+BLEUARTRX");
  ble.readline();
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    return;
  }
  // Some data was found, its in the buffer
  Serial.print(F("[Recv] ")); Serial.println(ble.buffer);
  ble.waitForOK();

}

void printGyro()
{
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.println(" deg/s");
#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif
}

void printAccel()
{
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");
#elif defined PRINT_RAW
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif

}

void printMag()
{
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz), 2);
  Serial.println(" gauss");
#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
#endif
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);

  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
}

void testforehand()
{

  // my part of code starts here, first value is  <-3,2nd is >+3 (for forehand), 3rd is >1

  if ((imu.calcGyro(imu.gz)) > 1) {   //used to be 1
    delay(200);
    imu.readGyro();
    if ((imu.calcGyro(imu.gx)) > -5 ) {   // used to be -3
      delay(200);
      imu.readGyro();
      if ((imu.calcGyro(imu.gy)) < 3) {   // used to be 3
        delay(200);
        Serial.println("FOREHAND");
        imu.readGyro();
        //testforehand();
      }
    }
    forehandstat++;
    //break;
  } else {
    Serial.print("TOTAL # OF FOREHANDS "); Serial.println(forehandstat);
    waiting();
  }   // my part of code ends here
}

void testbackhand()
{

  // my part of code starts here, first value is  <-3,2nd is >+3 (for forehand), 3rd is >1

  if ((imu.calcGyro(imu.gz)) > 1) { //used to be 1
    delay(200);
    imu.readGyro();
    if ((imu.calcGyro(imu.gx)) < -5 ) {   // used to be -3
      delay(200);
      imu.readGyro();
      if ((imu.calcGyro(imu.gy)) > 3) {   // used to be 3
        delay(200);
        Serial.println("BACKHAND");
        imu.readGyro();
        //testbackhand();
      }
    }
    backhandstat++;
    //break;
  }
  else {
    Serial.print("TOTAL # OF BACKHANDS "); Serial.println(backhandstat);
    waiting();
  }   // my part of code ends here
}


void waiting()
{
  Serial.println("NO MOVEMENT");

  ble.print("AT+BLEUARTTX=");
  ble.readline();
  //ble.println(backhandstat);
  //ble.print("AT+BLEUARTTX=");
  //ble.readline();
  //ble.println("NO MOVEMENT\n");
  ble.println("\n");
  delay(1000);
}


void testmove()
{
  if ((imu.calcGyro(imu.gz)) > 2){
  delay(200);
  imu.readGyro();
    if ((imu.calcGyro(imu.gx)) > -5 ) {   // forehand test starts here
      delay(200);
      imu.readGyro();
      if ((imu.calcGyro(imu.gy)) < 3) {
        delay(200);
        ble.print("AT+BLEUARTTX=");
 // ble.readline();
  ble.println(forehandstat);
  ble.print("AT+BLEUARTTX=");
 // ble.readline();
  ble.println(" : FOREHAND");
        //Serial.println("FOREHAND");
        forehandstat++;
        //Serial.println(forehandstat);
        //waiting();
        delay(1000);
        imu.readGyro();
      }
    }
    else{   // if it was not backhand, 'else' test for backhand
      if ((imu.calcGyro(imu.gx)) < -5 ) {
        delay(200);
        imu.readGyro();
        if ((imu.calcGyro(imu.gy)) > 3) {
        delay(200);
        //Serial.println("BACKHAND");
        ble.print("AT+BLEUARTTX=");
        //ble.readline();
        ble.println("   BACKHAND:n");
        //ble.readline();
        ble.print("AT+BLEUARTTX=");
        //ble.readline();
        ble.println("-----");
        ble.print("AT+BLEUARTTX=");
        //ble.readline();
        ble.println(backhandstat);
        backhandstat++;
        //Serial.println(backhandstat);
        //waiting();
        delay(1000);
        imu.readGyro();
      }
        
      }
    }
  }
}
//---------IMU STUFF ENDS HERE--------

//------BLUETOOTH STUFF STARTS HERE------
bool getUserInput(char buffer[], uint8_t maxSize)
{
  // timeout in 100 milliseconds
  TimeoutTimer timeout(100);

  memset(buffer, 0, maxSize);
  while( (!Serial.available()) && !timeout.expired() ) { delay(1); }

  if ( timeout.expired() ) return false;

  delay(2);
  uint8_t count=0;
  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && (Serial.available()) );

  return true;
}
