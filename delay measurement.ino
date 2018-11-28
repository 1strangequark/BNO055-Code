#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
unsigned long time;

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  Serial.flush();
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  Serial.print("Time: ");
  time = millis();

  Serial.println(time);
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  
  /* Euler Angle Data */
//  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//  Serial.print(euler.x());
//  Serial.print(",");
//  Serial.print(euler.y());
//  Serial.print(",");
//  Serial.print(euler.z());
//  Serial.print(",");

/* Acceleration data*/
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER );
  Serial.print(accel.x());  //x acceleration
  Serial.print(",");
  Serial.print(accel.y()); //y accel
  Serial.print(",");
  Serial.print(accel.z()); //z accel
  Serial.print(",");

/* Gyroscope data*/  
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE );
  Serial.print(gyro.x());  //x gyro
  Serial.print(",");
  Serial.print(gyro.y()); //y gyro
  Serial.print(",");
  Serial.print(gyro.z());//z gyro
  Serial.print(",");

/* Quaternion data*/ 
  imu::Quaternion quat = bno.getQuat();
  Serial.print(quat.w(), 4);
  Serial.print(",");
  Serial.print(quat.y(), 4);
  Serial.print(",");
  Serial.print(quat.x(), 4);
  Serial.print(",");
  Serial.print(quat.z(), 4);
  
//  Serial.println("\t\t");

  /* Display calibration status for each sensor. */
//  uint8_t system, gyroC, accelC, magC = 0;
//  bno.getCalibration(&system, &gyroC, &accelC, &magC);
//  Serial.print("CALIBRATION: Sys=");
//  Serial.print(system, DEC);
//  Serial.print(" Gyro=");
//  Serial.print(gyroC, DEC);
//  Serial.print(" Accel=");
//  Serial.print(accelC, DEC);
//  Serial.print(" Mag=");
//  Serial.println(magC, DEC);
  delay(10);

  Serial.println("\t\t");
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
