
#ifndef ULTRASONIC_SENSORS_H
#define ULTRASONIC_SENSORS_H


#define TRIG_45     23
#define ECHO_45     26

#define TRIG_Front  32
#define ECHO_Front  25

#define TRIG_Back   5  // 4
#define ECHO_Back   14  // 18

#define TRIG_Left   4   // 5
#define ECHO_Left   18  // 14
#ifdef robot
  #define TRIG_Back   5  // 4
  #define ECHO_Back   14  // 18

  #define TRIG_Left   4   // 5
  #define ECHO_Left   18  // 14
#else 
  #define TRIG_Back  4
  #define ECHO_Back  18

  #define TRIG_Left  5
  #define ECHO_Left  14
#endif

#define TRIG_Right  13
#define ECHO_Right  19

#define MAX_DISTANCE 400  // cm

NewPing sonar_45(TRIG_45, ECHO_45, MAX_DISTANCE);
NewPing sonar_Front(TRIG_Front, ECHO_Front, MAX_DISTANCE);
NewPing sonar_Back(TRIG_Back, ECHO_Back, MAX_DISTANCE);
NewPing sonar_Left(TRIG_Left, ECHO_Left, MAX_DISTANCE);
NewPing sonar_Right(TRIG_Right, ECHO_Right, MAX_DISTANCE);

struct UltrasonicData {
  unsigned int front;
  unsigned int back;
  unsigned int left;
  unsigned int right;
  unsigned int angle45;
};

UltrasonicData ultrasonicReadings;

void getUSReadings();
void returnUSReadings(int &Front,int &Back,int &Left,int &Right,int &Angle45);

void getUSReadings() {
  delay(5);
  ultrasonicReadings.angle45 = sonar_45.ping_cm();
  delay(5);
  ultrasonicReadings.front = sonar_Front.ping_cm();
  delay(5);
  ultrasonicReadings.back = sonar_Back.ping_cm();
  delay(5);
  ultrasonicReadings.left = sonar_Left.ping_cm();
  delay(5);
  ultrasonicReadings.right = sonar_Right.ping_cm();
  if(ultrasonicReadings.angle45 == 0)
  ultrasonicReadings.angle45 = 999;
  if(ultrasonicReadings.front == 0)
  ultrasonicReadings.front = 999;
  if(ultrasonicReadings.back == 0)
  ultrasonicReadings.back = 999;
  if(ultrasonicReadings.left == 0)
  ultrasonicReadings.left = 999;
  if(ultrasonicReadings.right == 0)
  ultrasonicReadings.right = 999;
}

void returnUSReadings(int &Front,int &Back,int &Left,int &Right,int &Angle45) {
  Front = ultrasonicReadings.front;
  Back = ultrasonicReadings.back;
  Left = ultrasonicReadings.left;
  Right = ultrasonicReadings.right;
  Angle45 = ultrasonicReadings.angle45;
}

#endif
