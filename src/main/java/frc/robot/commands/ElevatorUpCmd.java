/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorUpCmd extends Command {
  public enum Mode{ PID, SPEED };
  private Mode mode;

  public ElevatorUpCmd(Mode controlMode) {
    requires (Robot.pidElevator);
    mode = controlMode;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    switch(mode){
      case PID:
        double newPos = Robot.pidElevator.getTargetPosition() + RobotMap.ElevatorUpPidDelta;
        Robot.pidElevator.setPIDPosition(newPos);
      break;
      case SPEED:
        Robot.pidElevator.Up();
      break;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
