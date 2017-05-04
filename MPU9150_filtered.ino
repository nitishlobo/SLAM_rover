// I2C device class (I2Cdev) demonstration Arduino sketch for MPU9150
// 1/4/2013 original by Jeff Rowberg <jeff@rowberg.net> at https://github.com/jrowberg/i2cdevlib
//          modified by Aaron Weiss <aaron@sparkfun.com>
//
//        Further modified for the purposes of the SLAM assignment by Mecheng 706 course group 10
//
// Changelog:
//     2011-10-07 - initial release
//     2013-1-4 - added raw magnetometer output

/* ============================================
I2Cdev device library code is placed under the MIT license

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "math.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
 #include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Includes extra library for filtering
#include "Filters.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float axf, ayf, azf;
float gxf, gyf, gzf;
float mzf, myf;

double absAngle, angOffset;

 // Setup Filter objects
    float filterFrequencyLow = 50.0;
    FilterOnePole lowpassFilterAx( LOWPASS, filterFrequencyLow );
    //FilterOnePole lowpassFilterAy( LOWPASS, filterFrequencyLow );
    FilterOnePole lowpassFilterAz( LOWPASS, filterFrequencyLow );
    
    float filterFrequencyHigh = 1.0;
    //FilterOnePole highpassFilterGx( HIGHPASS, filterFrequencyHigh );
    FilterOnePole highpassFilterGy( HIGHPASS, filterFrequencyHigh );
    //FilterOnePole highpassFilterGz( HIGHPASS, filterFrequencyHigh );
    
    FilterOnePole lowpassFilterMagz( LOWPASS, 80.0 );
    FilterOnePole lowpassFilterMagy( LOWPASS, 80.0 );


#define LED_PIN 13
bool blinkState = false;

//Setup velocity and position calc parameters
double prevMillis = 0, currentMillis = 0;
float dt;
float fwdVelPrev = 0, rightVelPrev = 0;
float fwdAccPrev = 0, rightAccPrev = 0;
float fwdVel, rightVel, fwdPosChange, rightPosChange;

float angVel, angPosChange;
float angAccPrev = 0, angVelPrev = 0, angPosPrev = 0;

float angPos = 0;

void MPU_setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize Serial1 communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    //Serial1.begin(115200);

    // initialize device
  //  Serial1.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
   // Serial1.println("Testing device connections...");
    //Serial1.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED for
    //pinMode(LED_PIN, OUTPUT);
}

void reset_MPU_Vel() {
  fwdVelPrev = 0;
  rightVelPrev = 0;
  angVelPrev = 0;
}

void set_heading_MPU(double newAngle){
    float curRef = Read_MPU();
    angOffset = newAngle - curRef;
}

float Read_MPU() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    axf = lowpassFilterAx.input( ax ) / 1670;
    //ayf = lowpassFilterAy.input( ay ) / 1670;
    azf = lowpassFilterAz.input( az ) / 1670;

    //gxf = highpassFilterGx.input( gx );
    gyf = highpassFilterGy.input( gy ) * 10;
    //gzf = highpassFilterGz.input( gz );

    mzf = lowpassFilterMagz.input( mz ) - 40;
    myf = lowpassFilterMagy.input( my ) - 20;

    absAngle = atan(mzf / myf) * (180.0 / PI) - 90;
    if(myf < 0){
      absAngle = absAngle + 180;
    }

    absAngle = absAngle + angOffset;
    if(absAngle > 180){
      absAngle = absAngle - 360;
    } else if(absAngle < -180){
      absAngle = absAngle + 360;
    }

    //Serial1.println(absAngle);

/*
    if (gyf < 80 && gyf > -80){
      gyf = 0;
    }
    
    display tab-separated accel/gyro x/y/z values
    Serial1.print("a/g/m:\t");
    Serial1.print(axf); Serial1.print("\t");
    //Serial1.print(ayf); Serial1.print("\t");
    Serial1.print(azf); Serial1.print("\t");
    //Serial1.print(gxf); Serial1.print("\t");
    Serial1.print(gyf); Serial1.print("\t");
    //Serial1.print(gzf); Serial1.print("\t");
    //Serial1.print(mx); Serial1.print("\t");
    Serial1.print(my); Serial1.println("\t");*/

    
    static float posChanges[3];

    //Calculates velocity of the rover
    currentMillis = millis();
    dt = (currentMillis - prevMillis) / 1000;
    fwdVel = fwdVelPrev + ((fwdAccPrev + ((azf - fwdAccPrev) / 2))*dt);
    rightVel = rightVelPrev + ((rightAccPrev + ((axf - rightAccPrev) / 2))*dt);

    //Calculates change in position of the rover
    fwdPosChange = (fwdVelPrev + ((fwdVel - fwdVelPrev) / 2)) * dt;
    rightPosChange = (rightVelPrev + ((rightVel - rightVelPrev) / 2)) * dt;

    //posChanges[0] = fwdPosChange;
    //posChanges[1] = rightPosChange;
    //posChanges[2] = absAngle;

//    Serial1.print(fwdPosChange); Serial1.print("\t");
//    Serial1.println(rightPosChange);
    
    //updates past values
    fwdVelPrev = fwdVel;
    rightVelPrev = rightVel;
    fwdAccPrev = azf;
    rightAccPrev = axf;
    prevMillis = currentMillis;

/*
    Serial1.print("dt/angPosChange/angVel/gyf/angPos:\t");
    Serial1.print(dt); Serial1.print("\t");
    Serial1.print(angPosChange); Serial1.print("\t");
    Serial1.print(angVel); Serial1.print("\t");
    Serial1.print(gyf); Serial1.print("\t");
    Serial1.println(angPos);
*/
    
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    return absAngle;
}


