/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  // indicate what we want to test
  public static enum TestModule {
    LimitSwitch,Elevator,Wrist,Arm,Encoder,Custom
  }
  public static TestModule testModule = TestModule.Encoder  ;//change to whatever you want to test meaning (TestModule.Elevator)

  // These are PIO ids

  //Encoder 1
  public static final int EncoderPIOId1 = 5;
  public static final int EncoderPIOId2 = 6;  

  public static final int ElevatorEncoderPIOId1 = 5;
  public static final int ElevatorEncoderPIOId2 = 6;  
  
  public static final int LimitSwitchPIOId1 = 0;
  public static final int LimitSwitchPIOId2 = 1;
  
  public static final int ElevatorTopLimitSwitchId = 0;
  public static final int ElevatorBottomLimitSwitchId = 1;
  
  public static final int ArmTopLimitSwitchId = 2;
  public static final int ArmBottomLimitSwitchId = 3;

  public static final int WristTopLimitSwitchId = 4;
  public static final int WristBottomLimitSwitchId = 5;

  // these are PWM ids
  public static final int SparkMotorId1 = 5;
  public static final int SparkMotorId2 = 6;

  // these are CAN bus ids
  public static final int TalonMotorCanId1 = 8;
  public static final int TalonMotorCanId2 = 7;
  public static final int TalonMotorCanID3 = 6;
  public static final int TalonMotorCanID4 = 5;
  public static final int TalonDriveLeftFront = 2;
  public static final int TalonDriveLeftBack = 3;
  public static final int TalonDriveRightFront = 1;
  public static final int TalonDriveRightRear = 4;
  public static final double TalonMinOutput = -0.25;
  public static final double TalonMaxOutput = 0.3;

  /// generic
  public static final double TalonUpSpeed = -.2;
  public static final double TalonUpPidDelta = -20; // match sign of upspeed
  public static final double TalonDownSpeed = .3;
  public static final double TalonDownPidDelta = 20; // match sign of down speed
  public static final double TalonStopSpeed = .001;
  public static final double TalonAbsTolerance = 5;
  public static final int    TalonCruiseSpeed = 15000;
  public static final int    TalonAcceleration = 6000;
  public static final double TalonPID_P = 4; //0.2;
  public static final double TalonPID_I = 0.0;
  public static final double TalonPID_D = 0.0;
  public static final double TalonPID_F = 0.0;


  //arm
  public static final double TalonArmMinOutput = -0.10;
  public static final double TalonArmMaxOutput = 0.3;
  public static final double TalonArmUpSpeed = .3;
  public static final double TalonArmUpPidDelta = 20; // match sign of upspeed
  public static final double TalonArmDownSpeed = -.2;
  public static final double TalonArmDownPidDelta = -20; // match sign of down speed
  public static final double TalonSrmStopSpeed = .001;
  public static final double TalonArmAbsTolerance = 2.5;
  public static final int    TalonArmCruiseSpeed = 15000;
  public static final int    TalonArmAcceleration = 6000;
  public static final double TalonArmInitPIDUp_P = 1.28; //0.2;
  public static final double TalonArmInitPIDDown_P = 1.28; //0.2;
  public static final double TalonArmMovePIDUp_P = 18; //0.2;
  public static final double TalonArmMovePIDDown_P = 18 ; //0.2;
  public static final double TalonArmPID_I = 0.0; //be careful when chnaging, speed increases by a ridiculous amount
  public static final double TalonArmPID_D = 0.0;
  public static final double TalonArmPID_F = 2.0;
  public static final double TalonArmCalibrateUpDist = 300;
  public static final double TalonArmCalibrateDownDist = -300;

  //wrist
  public static final double TalonWristUpSpeed = 0.3;
  public static final double TalonWristUpPidDelta = 20; // match sign of upspeed
  public static final double TalonWristDownSpeed = -0.3;
  public static final double TalonWristDownPidDelta = -20; // match sign of down speed
  public static final double TalonWristStopSpeed = .001;
  public static final double TalonWristAbsTolerance = 5;
  public static final int    TalonWristCruiseSpeed = 15000;
  public static final int    TalonWristAcceleration = 6000;
  public static final double TalonWristPID_P = 4; //0.2;
  public static final double TalonWristPID_I = 0.0;
  public static final double TalonWristPID_D = 0.0;
  public static final double TalonWristPID_F = 0.0;
  public static final double TalonWristCalibrateUpDist = -100;
  public static final double TalonWristCalibrateDownDist = 1000;

  // elevator values
  public static final int SparkElevatorId = 5;
  public static final double ElevatorUpSpeed = 0.4;
  public static final double ElevatorUpPidDelta = 0.5; // match sign of up speed
  public static final double ElevatorDownSpeed = -0.4;
  public static final double ElevatorDownPidDelta = -0.5; // match sign of down speed
  public static final double ElevatorStopSpeed = .001;
  public static final double ElevatorPID_P = 0.001;
  public static final double ElevatorPID_I = 0.0;
  public static final double ElevatorPID_D = 0.0;
  public static final double ElevatorPID_F = 0.0;
  public static final double ElevatorDistPerPulse = 1;
  public static final boolean ElevatorReverseDirection = false;
  public static final double ElevatorMinOutput = -.5;
  public static final double ElevatorMaxOutput = .5;
  public static final double ElevatorAbsTolerance = 5;
  public static final double ElevatorCalibrateUpDist = 5;
  public static final double ElevatorCalibrateDownDist = -50;

  // intake values
  public static final int SparkIntakeId = 4;
  public static final double IntakeIngressSpeed = -0.56;
  public static final double IntakeEgressSpeed = 0.8;
  public static final double IntakeStopSpeed = .001;

  // Joystick to use
  public static final int Joystick0Id = 0;
  public static final int Joystick1Id = 1;

  // these are controller button ids (on joystick)
  public static final int JoystickButtonA = 1;
  public static final int JoystickButtonB = 2;
  public static final int JoystickButtonX = 3;
  public static final int JoystickButtonY = 4;
  public static final int JoystickButtonShoulderLeft = 5;
  public static final int JoystickButtonShoulderRight = 6;
  public static final int JoystickButtonBack = 7;
  public static final int JoystickButtonStart = 8;
  
  // joystick axis mapping
  public static final int JoystickAxisSpeed = 0;
  public static final int JoystickAxisRotation = 1;
  public static final double JoystickDeadZone = 0.05;

  // these values are used to expand (value > 1)
  // keep the identity (value == 1) or reduce
  // (value < 1) the max and min speed the joystick
  // moves the robot  
  public static final double SpeedScaleFactor = 0.65;
  public static final double RotationScaleFactor = 0.45;

  // this is used to make the encoder phase check less sensitive
  public static final double EncoderSlop = 20;//used to be 2

  // position of elevator,arm and wrist to grab a ball
  public static final int    GrabBallPosIndex = 0;
  public static final double GrabBallElevatorPos = 10;
  public static final double GrabBallArmPos = 20;
  public static final double GrabBallWristPos = 100;

  // position of elevator,arm stow position
  public static final int    StowPosIndex = 1;
  public static final double StowBallElevatorPos = 20;
  public static final double StowBallArmPos = 100;
  public static final double StowBallWristPos = 200;
  
  // position of elevator,arm to put in low hole
  public static final int    LowBallPosIndex = 2;
  public static final double LowBallElevatorPos = 30;
  public static final double LowBallArmPos = 50;
  public static final double LowBallWristPos = 200;

  // position of elevator,arm to put in middle hole
  public static final int    MiddleBallPosIndex = 3;
  public static final double MiddleBallElevatorPos = 30;
  public static final double MiddleBallArmPos = 100;
  public static final double MiddleBallWristPos = 200;
  
  // position of elevator,arm to put in high hole
  public static final int    HighBallPosIndex = 4;
  public static final double HighBallElevatorPos = 30;
  public static final double HighBallArmPos = 150;
  public static final double HighBallWristPos = 200;

  // position of elevator,arm carry ball
  public static final int    CarryBallPosIndex = 5;
  public static final double CarryBallElevatorPos = 30;
  public static final double CarryBallArmPos = 100;
  public static final double CarryBallWristPos = 200;

  public static final int    MaxBallPosIndex = 6;

}
