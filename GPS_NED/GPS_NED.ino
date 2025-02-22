/* Author: Praveen
 * This code reads the latitude and longitude data 
 * from NEO-M8N GPS and converts them to a local
 * NED frame.
 */

#include <TinyGPS++.h>
#include <SoftwareSerial.h> 
#include <math.h>
#include "MPU9250.h"
//#include <MatrixMath.h>

// Arduino UNO PINS FOR SERIAL COMMUNICATION
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// NECESSARY CONSTANTS AND REFERENCE VALUES FOR GPS
const float DEG2RAD = PI/180;
const float RE_A = 6378137.0; // Semi-Major axis of Earth in m
const float ECC = 0.08181919; // First Eccentricity
const float LAT_REF  = 13.062259*DEG2RAD; // Reference Latitude 
const float LONG_REF = 80.197387*DEG2RAD; // Reference Longitude 
const float ALT_REF  = 11; // Reference altitude  

// GPS VARIABLES TO BE USED
float LAT, LONG, ALT, SAT, VEL, NE, PN[3], PE[3], PE_REF[3], ROT[3][3], PN_HALF[3];

// NECESSARY CONSTANTS FOR IMU
const float ACCX_BIAS = 0.274438;
const float ACCY_BIAS = -0.0664114;
const float ACCZ_BIAS = 0.0946291;

// IMU VARIABLES TO BE USED
float ACCX, ACCY, ACCZ, POS_IMU[2] = {0, 0}, VEL_IMU[2]= {0, 0};
float ACCELEROMETER[3], GYROSCOPE[3];
unsigned long OLD_TIME_IMU, NEW_TIME_IMU, dt;

// The TinyGPS++ OBJECT
TinyGPSPlus gps;

// MPU9250 OBJECT WITH THE SENSOR ON I2C BUS 0 WITH ADDRESS 0x68
MPU9250 IMU(Wire,0x68);

// ESTABLISHING SERIAL CONNECTION TO THE GPS DEVICE
SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
  IMU.begin();
  OLD_TIME_IMU = millis()/1000;
}

void loop() {
  for (int i = 0; i < 10; i++){
    IMU.readSensor();
    ACCX = IMU.getAccelX_mss() - ACCX_BIAS;
    ACCY = IMU.getAccelY_mss() - ACCY_BIAS;
    ACCZ = IMU.getAccelZ_mss();// - ACCZ_BIAS;
    
    NEW_TIME_IMU = millis()/1000;
    dt = NEW_TIME_IMU - OLD_TIME_IMU; 
    OLD_TIME_IMU = NEW_TIME_IMU;

    VEL_IMU[0] = VEL_IMU[0] + ACCX*dt;
    VEL_IMU[1] = VEL_IMU[1] + ACCY*dt;

    POS_IMU[0] = POS_IMU[0] + VEL_IMU[0]*dt + 0.5*ACCX*dt*dt;
    POS_IMU[1] = POS_IMU[1] + VEL_IMU[1]*dt + 0.5*ACCY*dt*dt;
    
    Serial.print(POS_IMU[0],6);
    Serial.print("\t");
    Serial.print(POS_IMU[1],6);
    Serial.print("\t");
    Serial.print(VEL_IMU[0],6);
    Serial.print("\t");
    Serial.print(VEL_IMU[1],6);
    Serial.print("\n");
    //delay(50);
    }
  //while (ss.available() > 0){
  for (int i = 0; i < 1; i++){  
    if (gps.encode(ss.read()))
      if (gps.location.isValid())
      {
        LAT  = gps.location.lat()*DEG2RAD;
        LONG = gps.location.lng()*DEG2RAD;
        
        if (gps.altitude.isValid()){
          ALT  = gps.altitude.meters();
          
          SAT  = gps.satellites.value();
          
          //ALT = 15.9;
          /*Serial.print("Lat= "); 
          Serial.print(LAT,6); 
          Serial.print("\t");
          Serial.print(" Long= ");
          Serial.print(LONG,6);
          Serial.print("\t");
          Serial.print(" Alt= ");
          Serial.print(ALT);
          Serial.print("\t");
          Serial.print("Satellites= ");
          Serial.print(SAT);
          Serial.print("\n");*/
      
          NE = RE_A/sqrt(1-(ECC*ECC)*sin(LAT)*sin(LAT));
      
          PE_REF[0] = (NE+ALT_REF)*cos(LAT_REF)*cos(LONG_REF);
          PE_REF[1] = (NE+ALT_REF)*cos(LAT_REF)*sin(LONG_REF);
          PE_REF[2] = (NE*(1-ECC*ECC)+ALT_REF)*sin(LAT_REF);
          
          PE[0] = (NE+ALT)*cos(LAT)*cos(LONG);
          PE[1] = (NE+ALT)*cos(LAT)*sin(LONG);
          PE[2] = (NE*(1-ECC*ECC)+ALT)*sin(LAT);
      
          ROT[0][0] = -sin(LAT_REF)*cos(LONG_REF);
          ROT[0][1] = -sin(LAT_REF)*sin(LONG_REF);
          ROT[0][2] = cos(LAT_REF);
          ROT[1][0] = -sin(LONG_REF);
          ROT[1][1] = cos(LONG_REF);
          ROT[1][2] = 0;
          ROT[2][0] = -cos(LAT_REF)*cos(LONG_REF);
          ROT[2][1] = -cos(LAT_REF)*sin(LONG_REF);
          ROT[2][2] = -sin(LAT_REF);
      
          PN_HALF[0] = PE[0] - PE_REF[0];
          PN_HALF[1] = PE[1] - PE_REF[1];
          PN_HALF[2] = PE[2] - PE_REF[2];
          PN[0] = ROT[0][0]*PN_HALF[0] + ROT[0][1]*PN_HALF[1] + ROT[0][2]*PN_HALF[2];
          PN[1] = ROT[1][0]*PN_HALF[0] + ROT[1][1]*PN_HALF[1] + ROT[1][2]*PN_HALF[2];
          PN[2] = ROT[2][0]*PN_HALF[0] + ROT[2][1]*PN_HALF[1] + ROT[2][2]*PN_HALF[2];
          
          Serial.print("X= "); 
          Serial.print(PN[0],6); 
          Serial.print("\t");
          Serial.print(" Y= ");
          Serial.print(PN[1],6);
          Serial.print("\t");
          Serial.print(" Z= ");
          Serial.print(PN[2],2);
          Serial.print("\n");
                 
         
          //if (gps.location.isUpdated()){
          /*Serial.print("Latitude= "); 
          Serial.print(gps.location.lat(), 6);
          Serial.print(" Longitude= "); 
          Serial.println(gps.location.lng(), 6);
          Serial.print("Number of satellites in use = "); 
          Serial.println(gps.satellites.value()); 
          //}*/
        }
      }         
  }
  PN[0] = 1.2;
  PN[1] = 1.1;
  
  POS_IMU[0] = POS_IMU[0]*0.1 + 0.9*PN[0]; 
  POS_IMU[0] = POS_IMU[1]*0.1 + 0.9*PN[1]; 
  
  Serial.print(POS_IMU[0],6);
  Serial.print("\t");
  Serial.print(POS_IMU[1],6);
  Serial.print("\n");
}
