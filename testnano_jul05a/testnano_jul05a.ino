#include <Adafruit_BNO055.h>
#include <SoftwareSerial.h>

// Remote data
int SteerPin = 11;
int ThrotPin = 10;
int GearPin  = 9;
int AUX1Pin  = 6;

//IMU data
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(2000000);
  pinMode(SteerPin,INPUT);
  pinMode(ThrotPin,INPUT);
  if (!bno.begin()){
    Serial.println("No BNO055 detected");
    delay(5000);
  }
}

void loop() {
  // Get remote data
  int SteerPWM = pulseIn(SteerPin, HIGH);
  int ThrotPWM = pulseIn(ThrotPin, HIGH);
  int GearPWM  = pulseIn(GearPin,  HIGH);
  int AUX1PWM  = pulseIn(AUX1Pin,  HIGH);

  // Get sensor data
  sensors_event_t orientationData , linearAccelData , angVelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER)      ;
  bno.getEvent(&angVelData     , Adafruit_BNO055::VECTOR_GYROSCOPE)  ;
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Post process IMU data
    // velocity of sensor in the direction it's facing
//  headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);
  
  // Print remote data 
  Serial.print(SteerPWM);
  Serial.print(',');
  Serial.print(ThrotPWM);
  Serial.print(',');
  Serial.print(GearPWM);
  Serial.print(',');
  Serial.print(AUX1PWM);

  // print orientation data
  Serial.print(',');
  Serial.print(orientationData.orientation.x);
  Serial.print(',');
  Serial.print(orientationData.orientation.y);
  Serial.print(',');
  Serial.print(orientationData.orientation.z);

  // print angular velocity data
  Serial.print(',');
  Serial.print(angVelData.gyro.x);
  Serial.print(',');
  Serial.print(angVelData.gyro.y);
  Serial.print(',');
  Serial.print(angVelData.gyro.z);

  // print linear acceleration data
  Serial.print(',');
  Serial.print(linearAccelData.acceleration.x);
  Serial.print(',');
  Serial.print(linearAccelData.acceleration.y);
  Serial.print(',');
  Serial.println(linearAccelData.acceleration.z);
}
