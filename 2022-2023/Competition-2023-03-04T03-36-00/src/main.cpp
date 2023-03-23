/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Quinn                                                     */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Code                                          */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Controller2          controller                    
// Rfront               motor         10              
// Lfront               motor         1               
// Lback                motor         2               
// Rback                motor         9               
// Flywheel             motor_group   3, 4            
// FlywheelSpeed        rotation      5               
// Indexer              digital_out   A               
// Intake               motor_group   11, 20          
// Endgame              digital_out   H               
// Gyro                 inertial      19              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the program has started and           */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

float speedmod = 1;//Drive Modifier
bool firing = false;//Rapid Fire (Depreciated)
float flspeed = 6; //This is 50% in volts, 1% is 0.12
bool temp = false;//Over Temp Flag
bool revrse = false;// Reverse Flag (Depreciated)

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  //Intake Speed
  Intake.setVelocity(100, percent);

  //Retract Indexer
  Indexer.set(true);
  Endgame.set(false);

  //Flywheel Coast to Prolong Lifetime
  Flywheel.setStopping(coast);

  //Brake for Defense
  Lfront.setStopping(brake);
  Rfront.setStopping(brake);
  Lback.setStopping(brake);
  Rback.setStopping(brake);
  

  

  
}

//Fire Pnumatics
void fire(){
  std::cout << FlywheelSpeed.velocity(rpm) << "\n";
  Indexer.set(false);
  wait(200, msec);
  Indexer.set(true);
  
 
  

  
 
}

//Settings
float speed = 1;
double kP = 0.1;
double kI = 0.1;
double kD = 0.1;
double turnkP = 0.7;
double turnkI = 00.1;
double turnkD = 0.1;
float flkD = 1 ;
float flkP = 0.00;
float flkI = 0.00;

int maxTurnIntegral = 300; // These cap the integrals
int maxIntegral = 300;
int maxFlIntegral = 300;
int integralBound = 3; //If error is outside the bounds, then apply the integral. This is a buffer with +-integralBound degrees

//Autonomous Settings
int desiredValue = 0; // These are where you say how far you want to go when using pid
int desiredTurnValue = 0;
float desiredFlSpeed = 0;
int error; //SensorValue - DesiredValue : Position
int prevError = 0; //Position 20 miliseconds ago
int derivative; // error - prevError : Speed
int totalError = 0; //totalError = totalError + error

int turnError; //SensorValue - DesiredValue : Position
int turnPrevError = 0; //Position 20 miliseconds ago
int turnDerivative; // error - prevError : Speed
int turnTotalError = 0; //totalError = totalError + error

float flError = 0;
float flPrevError = 0;
float flDerivitave = 0;
float flTotalError = 0;

float flDesiredValue = 0;

bool resetDriveSensors = false;

//Variables modified for use
bool enableDrivePID = true;

//Pasted from a C++ resource
double signnum_c(double x) {
  if (x > 0.0) return 1.0;
  if (x < 0.0) return -1.0;
  return x;
}

int drivePID(){
  
  while(enableDrivePID){

    //hone drive sensors 
    if (resetDriveSensors) {
      resetDriveSensors = false;
      
      
      Lfront.setPosition(0,degrees);
      Rfront.setPosition(0,degrees);

    }


    //Get the position of both motors
    int leftMotorPosition = Lfront.position(degrees);
    int rightMotorPosition = Rfront.position(degrees);

    ///////////////////////////////////////////
    //Lateral movement PID
    /////////////////////////////////////////////////////////////////////
    //Get average of the two motors
    int averagePosition = (leftMotorPosition + rightMotorPosition)/2;

    //Potential
    error = desiredValue - averagePosition ;

    //Derivative
    derivative = error - prevError;

    //Integral
    if(abs(error) < integralBound){
    totalError+=error; 
    }  else {
    totalError = 0;
    }
    //totalError += error;

    //This would cap the integral
    totalError = abs(totalError) > maxIntegral ? signnum_c(totalError) * maxIntegral : totalError;

    double lateralMotorPower = error * kP + derivative * kD;
    /////////////////////////////////////////////////////////////////////


    ///////////////////////////////////////////
    //Turning movement PID
    /////////////////////////////////////////////////////////////////////
    //Get average of the two motors
    int turnDifference = Gyro.rotation();
    //Potential
    turnError = turnDifference - desiredTurnValue;

    //Derivative
    turnDerivative = turnError - turnPrevError;

    //Integral
    if(abs(error) < integralBound){
    turnTotalError+=turnError; 
    }  else {
    turnTotalError = 0; 
    }
    //turnTotalError += turnError;

    //This would cap the integral
    turnTotalError = abs(turnTotalError) > maxIntegral ? signnum_c(turnTotalError) * maxIntegral : turnTotalError;

    double turnMotorPower = turnError * turnkP + turnDerivative * turnkD + turnTotalError * turnkI;
    /////////////////////////////////////////////////////////////////////

    //////////////////////////////////////
    //Flywheel PID
    ////////////////////////////////////////////////
    int spd = FlywheelSpeed.velocity(rpm);

    //Potential
    flError = flDesiredValue - spd ;

    //Derivative
    flDerivitave = flError - flPrevError;

    //Integral
    if(abs(flError) < integralBound){
    flTotalError+=flError; 
    }  else {
    flTotalError = 0;
    }
    //totalError += error;

    //This would cap the integral
    flTotalError = abs(totalError) > maxIntegral ? signnum_c(totalError) * maxIntegral : totalError;

    double FlMotorPower = error * kP + derivative * kD;

    

    


    ////////////////////////////////////////////////////////////

    //Set Voltages
    Lfront.spin(forward, lateralMotorPower*speed - turnMotorPower, voltageUnits::volt);
    Rfront.spin(forward, lateralMotorPower*speed + turnMotorPower, voltageUnits::volt);

    Lback.spin(forward, lateralMotorPower*speed - turnMotorPower, voltageUnits::volt);
    Rback.spin(forward, lateralMotorPower*speed + turnMotorPower, voltageUnits::volt);

    //Flywheel.spin(forward, FlMotorPower, volt);


    

    prevError = error;
    turnPrevError = turnError;

    //Output for debugging purposes
    std::cout <<  averagePosition << "," << Gyro.rotation() << ","  << desiredValue << "," << desiredTurnValue <<"," <<FlywheelSpeed.velocity(rpm) <<"\n";
    vex::task::sleep(20);

  }

  return 1;
}



//Stop Rapidfire (depreciated)
void stopfire() {
  Indexer.set(false);
}
  
//Slow Down Flywheel
void flslowdown (){
  if (flspeed > 0){
  flspeed -= 1.2;
  }
  Flywheel.spin(forward, flspeed,volt);
  Controller1.rumble("-");
  Controller2.rumble("-");
}

//Speed Up Flywheel
void flspeedup(){
  if (flspeed < 100){
  flspeed += 1.2;
  }
  Flywheel.spin(forward, flspeed,volt);
  Controller1.rumble(".");
  Controller2.rumble(".");
}


//Change Drive Multiplier up (depreciated)
void drspeedup(){
  if (speedmod < 1){
    speedmod += 0.15;
  }
}
//Change Drive Multiplier down (depreciated)
void drslowdown(){
 if (speedmod > 1){
   speedmod -= 0.15;
 }
}

//Driver Control
int DriverControl() {

  //Does Math to Figure out how to interperet Stick Positions as motor velocities

  while(true){
  /* Reverse Code (Depreciated)
  if (revrse) {
  Rfront.setVelocity((Controller1.Axis3.position() - (-Controller1.Axis4.position() +  Controller1.Axis1.position()))*speedmod, percent);
  Lfront.setVelocity((Controller1.Axis3.position() + (-Controller1.Axis4.position() + Controller1.Axis1.position()))*speedmod, percent);
  Rback.setVelocity((Controller1.Axis3.position() + (-Controller1.Axis4.position() - Controller1.Axis1.position()))*speedmod, percent);
  Lback.setVelocity((Controller1.Axis3.position() - (-Controller1.Axis4.position() - Controller1.Axis1.position()))*speedmod, percent);
  } else {*/
  Rfront.setVelocity((-Controller1.Axis3.position() - (Controller1.Axis4.position() +  Controller1.Axis1.position()))*speedmod, percent);
  Lfront.setVelocity((-Controller1.Axis3.position() + (Controller1.Axis4.position() + Controller1.Axis1.position()))*speedmod, percent);
  Rback.setVelocity((-Controller1.Axis3.position() + (Controller1.Axis4.position() - Controller1.Axis1.position()))*speedmod, percent);
  Lback.setVelocity((-Controller1.Axis3.position() - (Controller1.Axis4.position() - Controller1.Axis1.position()))*speedmod, percent);

  //}
  Rfront.spin(forward);
  Lfront.spin(forward);
  Rback.spin(forward);
  Lback.spin(forward);
  wait(20,msec);
  }
  return 0;
}

//Rotate Function (Depreciated) 
//See Turnto
void rotate(float rotation) {
  Lback.rotateFor(forward, rotation/2, degrees, false);
  Lfront.rotateFor(forward, rotation/2, degrees,false);
  Rback.rotateFor(forward, rotation/2, degrees, false);
  Rfront.rotateFor(forward, rotation/2, degrees);

}

//Print Data to Screen and Check Flywheel Temp
int Data(){
  while(true) {
    

    //Cont 2
    Controller2.Screen.clearScreen();
    Controller2.Screen.setCursor(1,1);
    //Controller2.Screen.print("Fl Temp: ");
    //Controller2.Screen.print(Flywheel.temperature(fahrenheit));
    Controller2.Screen.print(Flywheel.power());
    Controller2.Screen.setCursor(2,1);
    Controller2.Screen.print("Fl %: ");
    Controller2.Screen.print(flspeed);
    //Controller2.Screen.print(FlywheelSpeed.velocity(rpm)/Flywheel.velocity(rpm));
    //Controller2.Screen.setCursor(3,1);
    //Controller2.Screen.print("Flywheel RPM: ");
    //Controller2.Screen.print(FlywheelSpeed.velocity(rpm));
    //Controller2.Screen.setCursor(4,1);
    Controller2.Screen.print("Fl RPM: ");
    Controller2.Screen.print(FlywheelSpeed.velocity(rpm));

    //Now Cont 1
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("Flywheel Temp: ");
    Controller1.Screen.print(Flywheel.temperature(fahrenheit));
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print("Flywheel %: ");
    Controller1.Screen.print(flspeed);
    Controller1.Screen.setCursor(3,1);
    Controller1.Screen.print("Flywheel RPM: ");
    Controller1.Screen.print(FlywheelSpeed.velocity(rpm));
    Controller1.Screen.setCursor(4,1);
    //Controller1.Screen.print("Flywheel Temp: ");
    //Controller1.Screen.print(Flywheel.temperature());

    if (Flywheel.temperature(fahrenheit) > 125 && temp  == false) {
      temp = true;
      Controller1.rumble("..- -");
    }
    
   wait(200, msec);


  }
  return 0;
}



void endgame(){
  Endgame.set(true);
  
  
}



/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  int marker;

  //Drive forward and Spin Roller
  /*
  driveforward(50, 100);
  Intake.spinFor(reverse, 0.4, turns);
  */

  //Run Pid
  vex::task PID(drivePID);

  //Hone Robot Rotation
  Gyro.setRotation(0, degrees);

  //Hone internal wheel encoders
  resetDriveSensors = true;

  //Drive to roller
  desiredValue = -300;

  //Wait so that robot can get there
  wait(0.5,seconds);

  //Start Flywheel
  Flywheel.spin(forward,12,volt);

  //Spin Roller
  Intake.spinFor(reverse, 0.5,turns);

  //drive forward
  resetDriveSensors = true;
  desiredValue = 100;
  wait(0.25,sec);

  resetDriveSensors = true;
  desiredTurnValue = 1;
  
  //Wait Till Flywheel is up to speed
  while(FlywheelSpeed.velocity(rpm) < 2500){wait(1,msec);}

  //Delay for good measure
  wait(0.25,sec);

  //Fire 
  Indexer.set(false);
  wait(0.5,seconds);
  Indexer.set(true);

  //Wait Till Flywheel is up to speed
  while(FlywheelSpeed.velocity(rpm) < 2500){wait(1,msec);}

  // Wait for good measure
  wait(0.25,sec);

  //Fire
  Indexer.set(false);
  wait(0.5,seconds);
  Indexer.set(true);

  //Stop Flywheel
  Flywheel.spin(forward,0,volt);

  //Stops pid
  enableDrivePID = false;




  
  

  

  
  
  

    
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  //Intake.spin(forward);


  //Data
  task ws2 = task(Data);

  //Drivercontrol
  task ws1 = task(DriverControl);

  //Cont 1 Fire
  Controller1.ButtonA.pressed(fire);
  
  //Controller1.ButtonA.released(stopfire);

  //Speed Up Flywheel
  Controller1.ButtonUp.pressed(flspeedup);

  //Slow Down Flywheel
  Controller1.ButtonDown.pressed(flslowdown);

  //Secondary Speed Up Flywheel
  //Controller2.ButtonL1.pressed(flspeedup);

  //Secondary Slow Down Flywheel
  //Controller2.ButtonL2.pressed(flslowdown);

  //Secondary Fire
  Controller2.ButtonA.pressed(fire);
  //Controller2.ButtonA.released(stopfire);
  
  

  //Speed Up Fl
  Controller2.ButtonUp.pressed(flspeedup);

  //Slow Down Fl
  Controller2.ButtonDown.pressed(flslowdown);
 
  Controller1.ButtonLeft.pressed(endgame);
  Controller2.ButtonLeft.pressed(endgame);


  

  //I did this because why not
  

  int x = 0;
  int y = 0;
  int z = 0;

  Brain.Screen.setFillColor("green");
  while (y < 240){
  Brain.Screen.drawPixel(x, y);
  x += 2;
  if (x >= 480) {
    y+=1;
    if (z == 0){
      z = 1;
      x = 1;
    } else {
    x = 0;
    z = 0;
    }
  }
  }
  
  
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}


























































