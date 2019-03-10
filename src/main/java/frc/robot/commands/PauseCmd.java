/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class PauseCmd extends Command {
  private int count;
  private int executeCount = 0;
  private final int logMsgInterval = 10;

  public PauseCmd(int runCount) {
    count = runCount;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    executeCount = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    executeCount++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if((count % logMsgInterval) == 0){
      Robot.Log("Pausing " + executeCount + "/" + count);
    }
    return (executeCount == count);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
