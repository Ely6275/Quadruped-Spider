//NOTES
/*
- THETA2 SHOULD NEVER BE NEGATIVE, meaning the femur should never be moving BELOW the shoulder joint
- LEG 1 SERVO WRITES
  shoulder.write(toDeg(theta1) + shoulder_ZeroPos);
  femur.write(toDeg(theta2) + femur_ZeroPos);
  tibia.write(tibia_ZeroPos - toDeg(theta3));

- LEG 2 SERVO WRITES
  shoulder2.write(shoulder_ZeroPos2 - toDeg(theta1));
  femur2.write(toDeg(theta2) + femur_ZeroPos);
  tibia2.write(tibia_ZeroPos - toDeg(theta3));

- LEG 3 SERVO WRITES (SAME AS LEG 2)
  shoulder3.write(leg3.shoulder_ZeroPos - toDeg(leg3.theta1));
  femur3.write(toDeg(leg3.theta2) + leg3.femur_ZeroPos);
  tibia3.write(leg3.tibia_ZeroPos - toDeg(leg3.theta3));

- LEG 4 SERVO WRITES (SAME AS LEG 1)
  shoulder4.write(leg4.shoulder_ZeroPos + toDeg(leg4.theta1));
  femur4.write(toDeg(leg4.theta2) + leg4.femur_ZeroPos);
  tibia4.write(leg4.tibia_ZeroPos - toDeg(leg4.theta3));
*/

#include<math.h>
#include<Servo.h>

//Declare Servo Objects 
Servo shoulder;
Servo femur;
Servo tibia;

Servo shoulder2;
Servo femur2;
Servo tibia2;

Servo shoulder3;
Servo femur3;
Servo tibia3;

Servo shoulder4;
Servo femur4;
Servo tibia4;

//Define each servo to a pin on the board
int shoulderPin = 5, femurPin = 6, tibiaPin = 7;
int shoulderPin2 = 2, femurPin2 = 3, tibiaPin2 = 4;
int shoulderPin3 = 8, femurPin3 = 9, tibiaPin3 = 10;
int shoulderPin4 = 11, femurPin4 = 12, tibiaPin4 = 13;

const double pi = 3.1415926535897932384626433832795;

//declare referance frames of each leg 
double baseX, baseY, baseZ = 0;
double x_0_1, y_0_1, z_0_1;
double x_0_2, y_0_2, z_0_2;
double x_0_3, y_0_3, z_0_3;

//lengths of each link of the legs
double a1 = 79, a2 = 110, a3 = 138, a4 = 70;

//Reference frame of each leg angle (in degrees)
const int tibia_ZeroPos = 110;
const int femur_ZeroPos = 90;
const int shoulder_ZeroPos = 45;

const int tibia_ZeroPos2 = 135;
const int femur_ZeroPos2 = 90;
const int shoulder_ZeroPos2 = 180;

const int tibia_ZeroPos3 = 155;
const int femur_ZeroPos3 = 65;
const int shoulder_ZeroPos3 = 125;

const int tibia_ZeroPos4 = 110;
const int femur_ZeroPos4 = 110;
const int shoulder_ZeroPos4 = 45;

double theta1 = 0;
double theta2 = 0;
double theta3 = 0;

//2D array to store leg coordinates for each of the 4 legs
double jointLocations1[4][3] = { 
                                {baseX, baseY, baseZ},
                                {x_0_1, x_0_2, x_0_3}, 
                                {y_0_1, y_0_2, y_0_3}, 
                                {z_0_1, z_0_2, z_0_3}
                              };

double jointLocations2[4][3] = { 
                                {baseX, baseY, baseZ},
                                {x_0_1, x_0_2, x_0_3}, 
                                {y_0_1, y_0_2, y_0_3}, 
                                {z_0_1, z_0_2, z_0_3}
                              };

double jointLocations3[4][3] = { 
                                {baseX, baseY, baseZ},
                                {x_0_1, x_0_2, x_0_3}, 
                                {y_0_1, y_0_2, y_0_3}, 
                                {z_0_1, z_0_2, z_0_3}
                              };

double jointLocations4[4][3] = { 
                                {baseX, baseY, baseZ},
                                {x_0_1, x_0_2, x_0_3}, 
                                {y_0_1, y_0_2, y_0_3}, 
                                {z_0_1, z_0_2, z_0_3}
                              };

//Leg structure
struct leg {
  //attributes include xyz positions and angle of each leg
  int shoulder_ZeroPos;
  int femur_ZeroPos;
  int tibia_ZeroPos;
  
  double theta1;
  double theta2;
  double theta3;

  double jointLocations[4][3];

  // Constructor for leg
  leg( int shoulderZero,  int femurZero,  int tibiaZero, double t1, double t2, double t3, double arr[4][3]) {
    shoulder_ZeroPos = shoulderZero;
    femur_ZeroPos = femurZero;
    tibia_ZeroPos = tibiaZero;
    theta1 = t1;
    theta2 = t2;
    theta3 = t3;
    memcpy(jointLocations, arr, sizeof(jointLocations));  // Copy array passed into the constructor (arr) to the structure member jointLocations
  }

};

//declare leg objects and place in an array
leg leg1(shoulder_ZeroPos,  femur_ZeroPos,  tibia_ZeroPos,  theta1, theta2, theta3, jointLocations1);
leg leg2(shoulder_ZeroPos2, femur_ZeroPos2, tibia_ZeroPos2, theta1, theta2, theta3, jointLocations2);
leg leg3(shoulder_ZeroPos3, femur_ZeroPos3, tibia_ZeroPos3, theta1, theta2, theta3, jointLocations3);
leg leg4(shoulder_ZeroPos4, femur_ZeroPos4, tibia_ZeroPos4, theta1, theta2, theta3, jointLocations4);

leg legs[4] = {leg1, leg2, leg3, leg4};



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //attach servos to their respective pins
  shoulder.attach(shoulderPin);
  femur.attach(femurPin);
  tibia.attach(tibiaPin);

  shoulder2.attach(shoulderPin2);
  femur2.attach(femurPin2);
  tibia2.attach(tibiaPin2);

  shoulder3.attach(shoulderPin3);
  femur3.attach(femurPin3);
  tibia3.attach(tibiaPin3);

  shoulder4.attach(shoulderPin4);
  femur4.attach(femurPin4);
  tibia4.attach(tibiaPin4);

  pinMode(shoulderPin, OUTPUT);
  pinMode(femurPin, OUTPUT);
  pinMode(tibiaPin, OUTPUT);

  pinMode(shoulderPin2, OUTPUT);
  pinMode(femurPin2, OUTPUT);
  pinMode(tibiaPin2, OUTPUT);

  pinMode(shoulderPin3, OUTPUT);
  pinMode(femurPin3, OUTPUT);
  pinMode(tibiaPin3, OUTPUT);

  pinMode(shoulderPin4, OUTPUT);
  pinMode(femurPin4, OUTPUT);
  pinMode(tibiaPin4, OUTPUT);

  //write the spider to its default position
  tibia.write(leg1.tibia_ZeroPos);
  tibia2.write(leg2.tibia_ZeroPos);
  tibia3.write(leg3.tibia_ZeroPos);
  tibia4.write(leg4.tibia_ZeroPos);
  
  delay(10);

  femur.write(leg1.femur_ZeroPos);
  femur2.write(leg2.femur_ZeroPos);
  femur3.write(leg3.femur_ZeroPos);
  femur4.write(leg4.femur_ZeroPos);
  
  delay(10);

  shoulder.write(leg1.shoulder_ZeroPos);
  shoulder2.write(leg2.shoulder_ZeroPos);
  shoulder3.write(leg3.shoulder_ZeroPos);
  shoulder4.write(leg4.shoulder_ZeroPos);

  delay(10);

  ////TEST CODE/////

  /*x_0_3 = 327;
  y_0_3 = 0;
  z_0_3 = 0;

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg1);
  forwardKinematics(leg1); 
  
  inverseKinematics(x_0_3, y_0_3, z_0_3, leg2.theta1, leg2.theta2, leg2.theta3, leg2);
  forwardKinematics(leg2);

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg3.theta1, leg3.theta2, leg3.theta3, leg3);
  forwardKinematics(leg3);

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg4.theta1, leg4.theta2, leg4.theta3, leg4);
  forwardKinematics(leg4);

  writeLegs(leg1, leg2, leg3, leg4);

  delay(3000);

  x_0_3 = 150;
  y_0_3 = 150;
  z_0_3 = 30;

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg1);
  forwardKinematics(leg1); 
  
  inverseKinematics(x_0_3, y_0_3, z_0_3, leg2.theta1, leg2.theta2, leg2.theta3, leg2);
  forwardKinematics(leg2);

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg3.theta1, leg3.theta2, leg3.theta3, leg3);
  forwardKinematics(leg3);

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg4.theta1, leg4.theta2, leg4.theta3, leg4);
  forwardKinematics(leg4);

  writeLegs(leg1, leg2, leg3, leg4);

  delay(5000);

  x_0_3 = 150;
  y_0_3 = 150;
  z_0_3 = -100;

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg1);
  forwardKinematics(leg1); 
  
  inverseKinematics(x_0_3, y_0_3, z_0_3, leg2.theta1, leg2.theta2, leg2.theta3, leg2);
  forwardKinematics(leg2);

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg3.theta1, leg3.theta2, leg3.theta3, leg3);
  forwardKinematics(leg3);

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg4.theta1, leg4.theta2, leg4.theta3, leg4);
  forwardKinematics(leg4);

  

  //delay(3000);
  //printJointLocations(leg4);

  //delay(3000);
  //Serial.println("");*/
  
  /*x_0_3 = 200;
  y_0_3 = 200;
  z_0_3 = -70;

  inverseKinematics(x_0_3, y_0_3, z_0_3, theta1, theta2, theta3, leg1);
  forwardKinematics(leg1);

  shoulder.write(toDeg(leg1.theta1) + leg1.shoulder_ZeroPos);
  femur.write(toDeg(leg1.theta2) + leg1.femur_ZeroPos);
  tibia.write(leg1.tibia_ZeroPos - toDeg(leg1.theta3));

  
  delay(3000);
  printJointLocations(leg1);

  delay(3000);
  Serial.println("");
  
  x_0_3 = 0;
  y_0_3 = 100;
  z_0_3 = 0;

  inverseKinematics(x_0_3, y_0_3, z_0_3, theta1, theta2, theta3, leg2);
  forwardKinematics(leg2);

  shoulder2.write(leg2.shoulder_ZeroPos - toDeg(leg2.theta2));
  femur2.write(toDeg(leg2.theta2) + leg2.femur_ZeroPos);
  tibia2.write(leg2.tibia_ZeroPos - toDeg(leg2.theta3));
  
  delay(3000);
  printJointLocations(leg2);

  delay(3000);
  Serial.println("");*/

}

void loop() {
  // put your main code here, to run repeatedly:

  writeLegs(leg1, leg2, leg3, leg4);
}


//Function that writes each leg to their assigned angles
void writeLegs(leg leg1, leg leg2, leg leg3, leg leg4){

  /*shoulder.write(toDeg(leg1.theta1) + leg1.shoulder_ZeroPos);
  femur.write(toDeg(leg1.theta2) + leg1.femur_ZeroPos);
  tibia.write(leg1.tibia_ZeroPos - toDeg(leg1.theta3));

  shoulder2.write(leg2.shoulder_ZeroPos - toDeg(leg2.theta1));
  femur2.write(toDeg(leg2.theta2) + leg2.femur_ZeroPos);
  tibia2.write(leg2.tibia_ZeroPos - toDeg(leg2.theta3));

  shoulder3.write(leg3.shoulder_ZeroPos - toDeg(leg3.theta1));
  femur3.write(toDeg(leg3.theta2) + leg3.femur_ZeroPos);
  tibia3.write(leg3.tibia_ZeroPos - toDeg(leg3.theta3));

  shoulder4.write(leg4.shoulder_ZeroPos + toDeg(leg4.theta1));
  femur4.write(toDeg(leg4.theta2) + leg4.femur_ZeroPos);
  tibia4.write(leg4.tibia_ZeroPos - toDeg(leg4.theta3));*/

  tibia.write(leg1.tibia_ZeroPos - toDeg(leg1.theta3));
  tibia2.write(leg2.tibia_ZeroPos - toDeg(leg2.theta3));
  tibia3.write(leg3.tibia_ZeroPos - toDeg(leg3.theta3));
  tibia4.write(leg4.tibia_ZeroPos - toDeg(leg4.theta3));
  
  delay(500);

  femur.write(toDeg(leg1.theta2) + leg1.femur_ZeroPos);
  femur2.write(toDeg(leg2.theta2) + leg2.femur_ZeroPos);
  femur3.write(toDeg(leg3.theta2) + leg3.femur_ZeroPos);
  femur4.write(toDeg(leg4.theta2) + leg4.femur_ZeroPos);
  
  delay(10);

  shoulder.write(toDeg(leg1.theta1) + leg1.shoulder_ZeroPos);
  shoulder2.write(leg2.shoulder_ZeroPos - toDeg(leg2.theta1));
  shoulder3.write(leg3.shoulder_ZeroPos - toDeg(leg3.theta1));
  shoulder4.write(leg4.shoulder_ZeroPos + toDeg(leg4.theta1));

  delay(10);
}

//updates all joint locations
void updateJointLocations(double refLocations[4][3], leg &refLeg){
  //Base joints all equal 0
  refLeg.jointLocations[0][0] = 0;
  refLeg.jointLocations[0][1] = 0;
  refLeg.jointLocations[0][2] = 0;

  //Shoulder Joint
  refLeg.jointLocations[1][0] = refLocations[1][0];
  refLeg.jointLocations[1][1] = refLocations[1][1];
  refLeg.jointLocations[1][2] = refLocations[1][2];

  //Femur joint
  refLeg.jointLocations[2][0] = refLocations[2][0];
  refLeg.jointLocations[2][1] = refLocations[2][1];
  refLeg.jointLocations[2][2] = refLocations[2][2];

  //Tibia joint
  refLeg.jointLocations[3][0] = refLocations[3][0];
  refLeg.jointLocations[3][1] = refLocations[3][1];
  refLeg.jointLocations[3][2] = refLocations[3][2];

}

/*
 - Calculates the forward kinematics for one leg using that leg's joint angles
 - Paramater must be added to determine which leg's forward kinematics are being calculated
*/

void forwardKinematics(leg &refLeg){
  x_0_1 = a1*cos(refLeg.theta1);
  y_0_1 = a1*sin(refLeg.theta1);
  z_0_1 = 0;

  double P1 = a2*cos(refLeg.theta2);
  
  double x_1_2 = P1*cos(refLeg.theta1);
  double y_1_2 = P1*sin(refLeg.theta1);

  double H = sqrt(a2*a2 + a3*a3 - (2*a2*a3*cos(pi-refLeg.theta3)));

  double alpha1 = acos( (a3*a3 - a2*a2 - H*H) / (-2*a2*H));
  double alpha2 = refLeg.theta3 - alpha1;
  double alpha3 = alpha1 - refLeg.theta2;

  double P = H*cos(alpha3);
  double P2 = P - P1;

  double x_2_3 = P2*cos(refLeg.theta1);
  double y_2_3 = P2*sin(refLeg.theta1);

  double G1 = (-1*H)*sin(alpha3);
  double G2 = a2*sin(refLeg.theta2);

  x_0_2 = x_0_1 + x_1_2;
  y_0_2 = y_0_1 + y_1_2;
  z_0_2 = G2;

  x_0_3 = x_0_2 + x_2_3;
  y_0_3 = y_0_2 + y_2_3;
  z_0_3 = G1;

  //local variable to store all calculated values
  double refLocations[4][3] = { 
                                {baseX, baseY, baseZ},
                                {x_0_1, x_0_2, x_0_3}, 
                                {y_0_1, y_0_2, y_0_3}, 
                                {z_0_1, z_0_2, z_0_3}
                              };
 // double refArr[4][3];
 // memcpy(refArr, jointLocations, sizeof(jointLocations));
  updateJointLocations(refLocations, refLeg);
 
}

/*
 - Calculates the inverse kinematics for one leg using that leg's end effector target position
 - Paramater must be added to determine which leg's inverse kinematics are being calculated
*/
void inverseKinematics(double x_0_3, double y_0_3, double z_0_3, double theta1, double theta2, double theta3, leg &refLeg){

  //Handle case where leg cannot reach specified point by writing the leg to its zero position
  if( sqrt(x_0_3*x_0_3 + y_0_3*y_0_3) > (a1+a2+a3) ){

    //Zero change in angles from initial positions
    refLeg.theta1 = 0;
    refLeg.theta2 = 0;
    refLeg.theta3 = 0;

    Serial.println("Point is out of range. Please try again.");
  }

  else{

    theta1 = atan(y_0_3 / x_0_3); //in radians
    double x_0_1 = a1 * cos(theta1);

    double x_1_3 = x_0_3 - x_0_1;
    double P = x_1_3 / cos(theta1);

    double G = abs(z_0_3);
    double H = sqrt(G*G + P*P);

    double alpha1 = acos( (a3*a3 - a2*a2 - H*H) / (-2*a2*H));
    double alpha2 = acos( (a2*a2 - H*H - a3*a3) / (-2*H*a3));
    //Serial.println(toDeg(alpha2));
    double alpha3 = asin(G/H);

    if(z_0_3 < 0) theta2 = alpha1 - alpha3;
    else theta2 = alpha1 + alpha3; //in radians
    
    theta3 = alpha1 + alpha2; //in radians

    refLeg.theta1 = theta1;
    refLeg.theta2 = theta2;
    refLeg.theta3 = theta3;

  }
    
}

//convert radians to degrees
double toDeg(double angle){
  return angle * (180/pi);
}

//convert degrees to radians
double microsToDeg(int microsVal){
  double x = (microsVal-600);
  return round(x/10);
}

//convert degrees to microseconds
double degToMicros(double degValue){
  double x = (degValue*10)+600;
  return x;
}

//print joint locations to serial monitor
void printJointLocations(leg refLeg){
  Serial.println(toDeg(refLeg.theta1));
  Serial.println(toDeg(refLeg.theta2));
  Serial.println(toDeg(refLeg.theta3));

  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 3; j++){
      Serial.print(refLeg.jointLocations[i][j]);
      if(j == 2) Serial.println("");
      else Serial.print(", ");
    }
  }
}

//IN DEVELOPMENT//
//Sequence that makes the spider walk
void legWalkSequence(){

  x_0_3 = 150;
  y_0_3 = 150;
  z_0_3 = -70;

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg1);
  forwardKinematics(leg1); 

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg4);
  forwardKinematics(leg4); 

  shoulder.write(toDeg(leg1.theta1) + leg1.shoulder_ZeroPos);
  femur.write(toDeg(leg1.theta2) + leg1.femur_ZeroPos);
  tibia.write(leg1.tibia_ZeroPos - toDeg(leg1.theta3));

  delay(150);

  shoulder4.write(leg4.shoulder_ZeroPos + toDeg(leg4.theta1));
  femur4.write(toDeg(leg4.theta2) + leg4.femur_ZeroPos);
  tibia4.write(leg4.tibia_ZeroPos - toDeg(leg4.theta3));

  delay(1000);

  ////////////////////////////////////////////////////////////////////////////////////////////////////

  x_0_3 = 100;
  y_0_3 = 100;
  z_0_3 = 0;

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg1);
  forwardKinematics(leg1);

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg4);
  forwardKinematics(leg4); 

  shoulder.write(toDeg(leg1.theta1) + leg1.shoulder_ZeroPos);
  femur.write(toDeg(leg1.theta2) + leg1.femur_ZeroPos);
  tibia.write(leg1.tibia_ZeroPos - toDeg(leg1.theta3));

  delay(150);

  shoulder4.write(leg4.shoulder_ZeroPos + toDeg(leg4.theta1));
  femur4.write(toDeg(leg4.theta2) + leg4.femur_ZeroPos);
  tibia4.write(leg4.tibia_ZeroPos - toDeg(leg4.theta3));

  delay(1000);

  ////////////////////////////////////////////////////////////////////////////////////////////////////

  x_0_3 = 250;
  y_0_3 = 0;
  z_0_3 = 0;

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg1);
  forwardKinematics(leg1); 

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg4);
  forwardKinematics(leg4); 

  shoulder.write(toDeg(leg1.theta1) + leg1.shoulder_ZeroPos);
  femur.write(toDeg(leg1.theta2) + leg1.femur_ZeroPos);
  tibia.write(leg1.tibia_ZeroPos - toDeg(leg1.theta3));

  delay(150);

  shoulder4.write(leg4.shoulder_ZeroPos + toDeg(leg4.theta1));
  femur4.write(toDeg(leg4.theta2) + leg4.femur_ZeroPos);
  tibia4.write(leg4.tibia_ZeroPos - toDeg(leg4.theta3));

  delay(1000);

  ////////////////////////////////////////////////////////////////////////////////////////////////////

  x_0_3 = 250;
  y_0_3 = 0;
  z_0_3 = -70;

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg1);
  forwardKinematics(leg1); 

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg4);
  forwardKinematics(leg4); 

  shoulder.write(toDeg(leg1.theta1) + leg1.shoulder_ZeroPos);
  femur.write(toDeg(leg1.theta2) + leg1.femur_ZeroPos);
  tibia.write(leg1.tibia_ZeroPos - toDeg(leg1.theta3));

  delay(150);

  shoulder4.write(leg4.shoulder_ZeroPos + toDeg(leg4.theta1));
  femur4.write(toDeg(leg4.theta2) + leg4.femur_ZeroPos);
  tibia4.write(leg4.tibia_ZeroPos - toDeg(leg4.theta3));

  delay(1000);

  ////////////////////////////////////////////////////////////////////////////////////////////////////

  x_0_3 = 150;
  y_0_3 = 150;
  z_0_3 = -70;

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg1);
  forwardKinematics(leg1); 

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg4);
  forwardKinematics(leg4); 

  shoulder.write(toDeg(leg1.theta1) + leg1.shoulder_ZeroPos);
  femur.write(toDeg(leg1.theta2) + leg1.femur_ZeroPos);
  tibia.write(leg1.tibia_ZeroPos - toDeg(leg1.theta3));

  delay(150);

  shoulder4.write(leg4.shoulder_ZeroPos + toDeg(leg4.theta1));
  femur4.write(toDeg(leg4.theta2) + leg4.femur_ZeroPos);
  tibia4.write(leg4.tibia_ZeroPos - toDeg(leg4.theta3));

  delay(1000);

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  x_0_3 = 150;
  y_0_3 = 150;
  z_0_3 = -70;

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg2);
  forwardKinematics(leg2); 

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg3);
  forwardKinematics(leg3); 

  shoulder2.write(toDeg(leg2.theta1) + leg2.shoulder_ZeroPos);
  femur2.write(toDeg(leg2.theta2) + leg2.femur_ZeroPos);
  tibia2.write(leg2.tibia_ZeroPos - toDeg(leg2.theta3));

  delay(150);

  shoulder3.write(leg3.shoulder_ZeroPos + toDeg(leg3.theta1));
  femur3.write(toDeg(leg3.theta2) + leg3.femur_ZeroPos);
  tibia3.write(leg3.tibia_ZeroPos - toDeg(leg3.theta3));

  delay(1000);

  ////////////////////////////////////////////////////////////////////////////////////////////////////

  x_0_3 = 100;
  y_0_3 = 100;
  z_0_3 = 0;

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg2);
  forwardKinematics(leg2); 

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg3);
  forwardKinematics(leg3); 

  shoulder2.write(toDeg(leg2.theta1) + leg2.shoulder_ZeroPos);
  femur2.write(toDeg(leg2.theta2) + leg2.femur_ZeroPos);
  tibia2.write(leg2.tibia_ZeroPos - toDeg(leg2.theta3));

  delay(150);

  shoulder3.write(leg3.shoulder_ZeroPos + toDeg(leg3.theta1));
  femur3.write(toDeg(leg3.theta2) + leg3.femur_ZeroPos);
  tibia3.write(leg3.tibia_ZeroPos - toDeg(leg3.theta3));

  delay(1000);

  ////////////////////////////////////////////////////////////////////////////////////////////////////

  x_0_3 = 250;
  y_0_3 = 0;
  z_0_3 = 0;

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg2);
  forwardKinematics(leg2); 

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg3);
  forwardKinematics(leg3); 

  shoulder2.write(toDeg(leg2.theta1) + leg2.shoulder_ZeroPos);
  femur2.write(toDeg(leg2.theta2) + leg2.femur_ZeroPos);
  tibia2.write(leg2.tibia_ZeroPos - toDeg(leg2.theta3));

  delay(150);

  shoulder3.write(leg3.shoulder_ZeroPos + toDeg(leg3.theta1));
  femur3.write(toDeg(leg3.theta2) + leg3.femur_ZeroPos);
  tibia3.write(leg3.tibia_ZeroPos - toDeg(leg3.theta3));

  delay(1000);

  ////////////////////////////////////////////////////////////////////////////////////////////////////

  x_0_3 = 250;
  y_0_3 = 0;
  z_0_3 = -70;

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg2);
  forwardKinematics(leg2); 

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg3);
  forwardKinematics(leg3); 

  shoulder2.write(toDeg(leg2.theta1) + leg2.shoulder_ZeroPos);
  femur2.write(toDeg(leg2.theta2) + leg2.femur_ZeroPos);
  tibia2.write(leg2.tibia_ZeroPos - toDeg(leg2.theta3));

  delay(150);

  shoulder3.write(leg3.shoulder_ZeroPos + toDeg(leg3.theta1));
  femur3.write(toDeg(leg3.theta2) + leg3.femur_ZeroPos);
  tibia3.write(leg3.tibia_ZeroPos - toDeg(leg3.theta3));

  delay(1000);

  ////////////////////////////////////////////////////////////////////////////////////////////////////

  x_0_3 = 150;
  y_0_3 = 150;
  z_0_3 = -70;

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg2);
  forwardKinematics(leg2); 

  inverseKinematics(x_0_3, y_0_3, z_0_3, leg1.theta1, leg1.theta2, leg1.theta3, leg3);
  forwardKinematics(leg3); 

  shoulder2.write(toDeg(leg2.theta1) + leg2.shoulder_ZeroPos);
  femur2.write(toDeg(leg2.theta2) + leg2.femur_ZeroPos);
  tibia2.write(leg2.tibia_ZeroPos - toDeg(leg2.theta3));

  delay(150);

  shoulder3.write(leg3.shoulder_ZeroPos + toDeg(leg3.theta1));
  femur3.write(toDeg(leg3.theta2) + leg3.femur_ZeroPos);
  tibia3.write(leg3.tibia_ZeroPos - toDeg(leg3.theta3));

  delay(1000);
}