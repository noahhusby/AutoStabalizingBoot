#include <PID_v1.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <MadgwickAHRS.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <math.h>

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5

#define BUTTON_PIN 5
#define VESC_PIN 10

#define VESC_OUTPUT_MULTIPLIER 1023

Servo esc;

double pidIn, pidOut, pidSetpoint;
PID mPIDController(&pidIn, &pidOut, &pidSetpoint, 0.035, 0, 0, DIRECT);

Madgwick filter;
bool engageSystem = true;


void setupSensor()
{
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}


void setup() 
{
  
  Serial.begin(115200);

  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
    
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("[Warning] Unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }

  // helper to just set the default scaling we want, see above!
  setupSensor();

  pidSetpoint = 0;
  mPIDController.SetMode(AUTOMATIC); 

  filter.begin(1);

  esc.attach(VESC_PIN);
  esc.writeMicroseconds(1500);
}

void loop() 
{
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  unsigned long microsNow;
  lsm.read();  /* ask it to read in the data */ 

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp); 


  aix = a.acceleration.x;
  aiy = a.acceleration.y;
  aiz = a.acceleration.z;
  
  gix = g.gyro.x;
  giy = g.gyro.y;
  giz = g.gyro.z;

  ax = convertRawAcceleration(aix);
  ay = convertRawAcceleration(aiy);
  az = convertRawAcceleration(aiz);
  gx = convertRawGyro(gix);
  gy = convertRawGyro(giy);
  gz = convertRawGyro(giz);

  filter.updateIMU(gx, gy, gz, ax, ay, az);

  roll = filter.getRoll();
  // Checks for max/min, if trips, disables system
  if(roll > 45 || roll < -45) {
    if(engageSystem){
      engageSystem = false;
      pidOut = 0.0;
      Serial.println("[Error] Maximum Distance Fault Tripped - System Disabled");
    }
  }

  if(engageSystem) {
    
    Serial.print(roll);
    pidIn = -abs(roll);
    Serial.print(" | PID Out: ");
    mPIDController.Compute();

    double localCalculatedOut;
    if(roll > 0) {
      localCalculatedOut = abs(pidOut);
    } else {
      localCalculatedOut = -abs(pidOut);
    }

    if(localCalculatedOut > 1) {
      localCalculatedOut =  1;
    } else if(localCalculatedOut < -1) {
      localCalculatedOut = -1;
    }

    esc.writeMicroseconds(map(VESC_OUTPUT_MULTIPLIER * localCalculatedOut, 0 , 1023, 1000, 2000));
    
    Serial.print(localCalculatedOut);
    Serial.println( );

  }

  if(digitalRead(BUTTON_PIN)) {
    engageSystem = true;
  }
  delay(50);
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

