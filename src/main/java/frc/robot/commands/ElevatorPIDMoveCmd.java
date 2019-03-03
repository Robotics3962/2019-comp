/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ElevatorPIDMoveCmd extends Command {
    private double targetPos;

  public ElevatorPIDMoveCmd(double pos) {
    requires(Robot.pidElevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.pidElevator.setPIDPosition(targetPos);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.pidElevator.setPIDPosition(targetPos);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean done = Robot.pidElevator.onTarget();
    Robot.Log("Elevator PidMove done:" + done);
    return done;
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
