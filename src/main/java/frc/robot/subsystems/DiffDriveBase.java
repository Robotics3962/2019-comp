/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.RobotMap;
import frc.robot.commands.DiffDriveCommand;

public class DiffDriveBase extends Subsystem {
  private WPI_TalonSRX  leftFrontTalonSRX = null;
  private WPI_TalonSRX  leftRearTalonSRX = null;
  private WPI_TalonSRX  rightFrontTalonSRX = null;
  private WPI_TalonSRX  rightRearTalonSRX = null;
  private SpeedControllerGroup leftMotors = null;
  private SpeedControllerGroup rightMotors = null;
  private DifferentialDrive differentialDrive = null;

  public DiffDriveBase() {
    leftFrontTalonSRX = new WPI_TalonSRX(RobotMap.TalonDriveLeftFront);
    leftRearTalonSRX = new WPI_TalonSRX(RobotMap.TalonDriveLeftBack);
   
    rightFrontTalonSRX = new WPI_TalonSRX(RobotMap.TalonDriveRightFront);
    rightRearTalonSRX = new WPI_TalonSRX(RobotMap.TalonDriveRightRear);
  
    leftMotors = new SpeedControllerGroup(leftFrontTalonSRX, leftRearTalonSRX);
    rightMotors = new SpeedControllerGroup(rightFrontTalonSRX, rightRearTalonSRX);

    // tells the left side that is should be inverted so that we drive straight with each side having positive motor values.
    rightFrontTalonSRX.setInverted(true);
    rightRearTalonSRX.setInverted(true); 

    //Config all talons
    DiffConfigTalons(rightFrontTalonSRX);
    DiffConfigTalons(rightRearTalonSRX);
    DiffConfigTalons(leftFrontTalonSRX);
    DiffConfigTalons(leftRearTalonSRX);

    differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
  }
  
  public void DiffConfigTalons(WPI_TalonSRX talon)
  {
    //Tells the talon that the max output that it can give is between 1 and -1 which would mean full forward and full backward.
    //There is no allowance currently for anything in between
    talon.configPeakOutputForward(1,0);
    talon.configPeakOutputReverse(-1,0);
  
    //Tells the talon that it should current limit itself so that we don't blow a 40amp breaker.
    talon.configPeakCurrentLimit(40,0);
    talon.enableCurrentLimit(true);
    talon.configContinuousCurrentLimit(40,0);
    //The max output current is 40Amps for .25 of a second
    talon.configPeakCurrentDuration(250, 0);
  
    //Tells the talon that is should only appy 12 volts or less to the motor.
    talon.configVoltageCompSaturation(12,0);

    // invert the direction if necessary
    //talon.setInverted(false);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DiffDriveCommand());
  }

  public void setSpeedAndRotation(double speed, double rotation){
    differentialDrive.arcadeDrive(speed, rotation);
  }

  public void stop(){
    leftMotors.stopMotor();
    rightMotors.stopMotor();
  }
}
