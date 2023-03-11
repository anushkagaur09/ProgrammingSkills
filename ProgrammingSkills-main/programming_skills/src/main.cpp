/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// rightFront           motor         19              
// leftFront            motor         3               
// leftBack             motor         8               
// rightBack            motor         14              
// Intake               motor         9               
// Expansion            digital_out   A               
// flywheel             motor         11              
// Controller1          controller                    
// rightMiddle          motor         17              
// leftMiddle           motor         18              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

bool Controller1XY = true;

double fly_kp = 0.1; // how fast it increases
double fly_ki = 0.2; // how much offshoot/range of fluctuation
double fly_kd = 0.00005; // how many fluctuations are there
double speed_margin = 0;
double speed_marg_pct = 2;
bool flyescvar = false;
int speed_volt = 0;

//flywheel spin

void flywheel_spin_fwd(double flywheel_target_speed_pct) {
  
  flywheel.setVelocity(flywheel_target_speed_pct, pct);
  flywheel.spin(directionType::fwd);
}

//flywheel spin PID code
void flywheel_PID(double flywheel_target_speed_pct){
double averagevolt = 0;
double preverror = 0;
double errorsum = 0;
double error = 0;
double derivative = 0;
double flywheel_target_speed_volt = (flywheel_target_speed_pct/100)*12;
Controller1.Screen.setCursor(1,1);
Controller1.Screen.print("         ");
wait(20,msec);
 
 while (flyescvar == false) {
    averagevolt = flywheel.voltage();
    error = flywheel_target_speed_volt - averagevolt;
    derivative = preverror - error;
    errorsum += error;
    preverror = error;
    speed_margin = fabs((error/flywheel_target_speed_volt) * 100);
    speed_volt =  error * fly_kp + fly_ki * errorsum + fly_kd * derivative;
    wait(20,msec);
  
    if(speed_margin <= speed_marg_pct) {
      flyescvar = true;
    } else {
        flywheel.spin(forward, speed_volt, volt);
    }
    wait(20, msec);
  }
 Controller1.Screen.setCursor(3,9);
 Controller1.Screen.print("DONE");
 wait(20,msec);
 
 // Maintain the speed
 flywheel.spin(forward, speed_volt, volt);
}
bool flywheelStart = false;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Expansion.set(false);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
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
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
     /* //shoot out discs at low goal
      rightFront.spin(reverse);
      leftBack.spin(forward);
      wait(0.2, seconds);
      rightFront.stop();
      leftBack.stop();
      //shoot out to low goal
      Intake.setVelocity(100, percent);
      wait(0.25, seconds);
      flywheel_PID(90);
      wait(2, seconds);
      Intake.spin(forward);
      wait(3, seconds);
      flywheel.stop();
      Intake.stop();
      wait(0.5, seconds);
      */
      //move forward towards the roller
      rightFront.spin(reverse);
      leftFront.spin(forward);
      wait(0.55, seconds);
      rightFront.stop();
      leftFront.stop();
      wait(0.5, seconds);
      //roll the roller
      Intake.spinFor(forward, 100, degrees, true);
      
      //move back to original position
      rightFront.spin(forward);
      leftFront.spin(reverse);
      wait(0.95, seconds);
      rightFront.stop();
      leftFront.stop();
      //turn towards the other roller
     
      leftFront.spin(reverse);
      rightBack.spin(forward);
      wait(1.17, seconds);
      leftFront.stop();
      rightBack.stop();
      //move forward towards the roller
      rightFront.spin(reverse);
      leftFront.spin(forward);
      wait(1.9, seconds);
      rightFront.stop();
      leftFront.stop();
      wait(0.5, seconds);
      //spin the roller
      Intake.spinFor(forward, 100, degrees,true);
      //move back to original position
      rightFront.spin(forward);
      leftFront.spin(reverse);
      wait(1.4, seconds);
      rightFront.stop();
      leftFront.stop();
      //turn back towards original position
      rightFront.spin(forward);
      leftBack.spin(reverse);
      wait(0.45, seconds);
      rightFront.stop();
      leftBack.stop();
      /*//turn 
      rightFront.spin(forward);
      leftBack.spin(reverse);
      wait(0.383, seconds);
      rightFront.stop();
      leftBack.stop();
      //move forward towards final turning position
      wait(0.5, seconds);
      rightFront.setVelocity(70, percent);
      leftFront.setVelocity(70, percent);
      rightFront.spin(forward);
      leftFront.spin(reverse);
      wait(1.22, seconds);
      rightFront.stop();
      leftFront.stop();
      //turn to face the high goal
      wait(0.5, seconds);
      rightFront.spin(reverse);
      leftBack.spin(forward);
      wait(0.6, seconds);
      rightFront.stop();
      leftBack.stop();
      //shoot out the discs
      Intake.setVelocity(100, percent);
      wait(2, seconds);
      flywheel_PID(60);
      wait(2, seconds);
      Intake.spin(forward);
      wait(4, seconds);
      flywheel.stop();
      Intake.stop();
      //turn 
      wait(0.5, seconds);
      rightFront.spin(forward);
      leftBack.spin(reverse);
      wait(0.6, seconds);
      rightFront.stop();
      leftBack.stop();
      //move back
      */
      wait(0.5, seconds);
      rightFront.setVelocity(70, percent);
      leftFront.setVelocity(70, percent);
      rightFront.spin(reverse);
      leftFront.spin(forward);
      wait(1.22, seconds);
      rightFront.stop();
      leftFront.stop();
      //shoot out expansion
      Expansion.set(true);   
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
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
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
