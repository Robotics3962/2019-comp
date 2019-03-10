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
  private int maxCount;
  private int executeCount = 0;
  private final int logMsgInterval = 10;

  public PauseCmd(int runCount) {
    maxCount = runCount;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    executeCount = 0;
    Robot.Log("starting pause command");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if((executeCount % logMsgInterval) == 0){
      //Robot.Log("Pausing " + executeCount + "/" + maxCount);
    }
    executeCount++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if( executeCount >= maxCount){
      Robot.Log("done with pause command");
    }
    return (executeCount >= maxCount);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    executeCount = maxCount;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.Log("pause interrupted");
    end();
  }
}
