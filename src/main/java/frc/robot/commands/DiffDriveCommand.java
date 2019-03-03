/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

import java.lang.Math;

public class DiffDriveCommand extends Command {
  double driveValue;
  double speedValue;

  public DiffDriveCommand() {
    requires(Robot.diffDriveBase);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    // the sign on the values from the joystick are opposite
    // the intuitive directions.  For example, pushing joystick 
    // forward is a negative number, which moves the motor backwards
    // so we invert the sign of the values
    double speed = Robot.m_oi.getLeftJoystickYVal() * -1;
    double rotation = Robot.m_oi.getLeftJoystickXVal() * 1;
    double scaledSpeed;
    double scaledRotation;
    
    scaledSpeed = speed * RobotMap.SpeedScaleFactor;
    scaledRotation = rotation * RobotMap.RotationScaleFactor;
    //scaledSpeed = scaleValue(speed, RobotMap.SpeedScaleFactor);
    //scaledRotation = scaleValue(rotation, RobotMap.RotationScaleFactor);

    Robot.diffDriveBase.setSpeedAndRotation(scaledRotation, scaledSpeed);//orig Scaledspeed, ScaledRotation
    
    //Logging for Debug
    //System.out.println(speed + ":" + scaledSpeed + "  " + rotation + ":" + scaledRotation);
    //System.out.println(scaledSpeed + ":" + scaledRotation);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
      // stop motors
      Robot.diffDriveBase.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  double scaleValue(double val, double scaleFactor){
    double abs = Math.abs(val);
    double tmp = abs * scaleFactor;
    double retVal = Math.copySign(tmp, val);
    return retVal;
  }
}
