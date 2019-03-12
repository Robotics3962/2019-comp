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

public class ElevatorDownCmd extends Command {
  public enum Mode{ PID, SPEED };
  private Mode mode = Mode.SPEED;

  public ElevatorDownCmd(Mode controlMode) {
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
        double newPos = Robot.pidElevator.getTargetPosition() + RobotMap.ElevatorDownPidDelta;
        Robot.pidElevator.setPIDPosition(newPos);
      break;
      case SPEED:
        Robot.pidElevator.Down();
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
    if(mode == Mode.SPEED){
      Robot.pidElevator.Stop();
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
