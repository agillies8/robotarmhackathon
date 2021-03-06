#include <SpeedyStepper.h> //https://github.com/Stan-Reifel/SpeedyStepper/blob/master/Documentation.pdf
//#include #include <AccelStepper.h>

// Connections to driver mks1.4
#define dirPin1  A1  // Direction for axis 1
#define stepPin1  A0// Step for axis 1
#define EnaPin1  38// Step for axis 1
#define dirPin2  A7  // Direction for axis 2
#define stepPin2  A6 // Step for axis 2
#define EnaPin2  A2// Step for axis 1
#define dirPin3  48  // Direction for axis 3
#define stepPin3  46 // Step for axis 3
#define EnaPin3  A8// Step for axis 1


float axis_1_pos = 0;
float axis_2_pos = 0;
float axis_3_pos = 0;

SpeedyStepper axis1;
SpeedyStepper axis2;
SpeedyStepper axis3;

float axis_1_move_dist = 0.1;
float axis_2_move_dist = 0.1;
float axis_3_move_dist = 0.1;

//AccelStepper axis1(1, stepPin1, dirPin1);
//AccelStepper axis2(1, stepPin2, dirPin2);
//AccelStepper axis3(1, stepPin3, dirPin3);

void setup() {

  //enable axis
pinMode(EnaPin1, OUTPUT);
pinMode(EnaPin2, OUTPUT);
pinMode(EnaPin3, OUTPUT);

digitalWrite(EnaPin1, 0);
digitalWrite(EnaPin2, 0);
digitalWrite(EnaPin3, 0);
  
  // Setup the steppers with speedy stepper lib
  axis1.connectToPins(stepPin1, dirPin1);
  axis2.connectToPins(stepPin2, dirPin2);
  axis3.connectToPins(stepPin3, dirPin3);

//  // set the speed and acceleration rates for the stepper motor
  axis1.setStepsPerRevolution(3200);
  axis1.setSpeedInRevolutionsPerSecond(0.5);
  axis1.setAccelerationInRevolutionsPerSecondPerSecond(0.5);
  
  axis2.setStepsPerRevolution(3200);
  axis2.setSpeedInRevolutionsPerSecond(0.5);
  axis2.setAccelerationInRevolutionsPerSecondPerSecond(0.5);

  axis3.setStepsPerRevolution(3200);
  axis3.setSpeedInRevolutionsPerSecond(0.5);
  axis3.setAccelerationInRevolutionsPerSecondPerSecond(0.5);

  // set the speed and acceleration rates for the stepper motor
//  axis1.setMaxSpeed(500.0);
//  axis1.setAcceleration(1000.0);
//  
//  axis2.setMaxSpeed(500.0);
//  axis2.setAcceleration(1000.0);
//
//  axis3.setMaxSpeed(500.0);
//  axis3.setAcceleration(1000.0);


  Serial.begin(9600);

}
void loop() {

  axis1.setupRelativeMoveInRevolutions(axis_1_move_dist);
  axis2.setupRelativeMoveInRevolutions(axis_2_move_dist);
  axis3.setupRelativeMoveInRevolutions(axis_3_move_dist);

  while((!axis1.motionComplete()) || (!axis2.motionComplete()) || (!axis2.motionComplete()))
    {
      axis1.processMovement();
      axis2.processMovement();
      axis3.processMovement();

    float joint1 = axis1.getCurrentPositionInRevolutions();
    float joint2 = axis2.getCurrentPositionInRevolutions();
    float joint3 = axis3.getCurrentPositionInRevolutions();

//    Serial.print("Axis 1: ");
//    Serial.print(joint1);
//    Serial.print("\t");
//    
//    Serial.print("Axis 2: ");
//    Serial.print(joint2);
//    Serial.print("\t");
//
//    Serial.print("Axis 3: ");
//    Serial.println(joint3);
    }

  Serial.println("NEXT");
  delay(1000);



  axis1.setupRelativeMoveInRevolutions(-1*axis_1_move_dist);
  axis2.setupRelativeMoveInRevolutions(-1*axis_2_move_dist);
  axis3.setupRelativeMoveInRevolutions(-1*axis_3_move_dist);

  while((!axis1.motionComplete()) || (!axis2.motionComplete()) || (!axis3.motionComplete()))
    {
      axis1.processMovement();
      axis2.processMovement();
      axis3.processMovement();

    float joint1 = axis1.getCurrentPositionInRevolutions();
    float joint2 = axis2.getCurrentPositionInRevolutions();
    float joint3 = axis3.getCurrentPositionInRevolutions();

//    Serial.print("Axis 1: ");
//    Serial.print(joint1);
//    Serial.print("\t");
//    
//    Serial.print("Axis 2: ");
//    Serial.print(joint2);
//    Serial.print("\t");
//
//    Serial.print("Axis 3: ");
//    Serial.println(joint3);
    }

  delay(1000);
}
