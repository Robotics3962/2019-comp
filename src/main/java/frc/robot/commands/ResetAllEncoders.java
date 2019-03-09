/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.ResetArmEncoderCmd;
import frc.robot.commands.ResetElevatorEncoderCmd;
import frc.robot.commands.ResetWristEncoderCmd;
import frc.robot.Robot;

public class ResetAllEncoders extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ResetAllEncoders() {
    requires(Robot.pidElevator);
    requires(Robot.encodedArmTalon);
    requires(Robot.encodedWristTalon);
    
    // this is emergency use only, only call this
    // command when the robot elevator, arm, and wrist
    // are in their zero positions. It will need to be
    // moved manually to their zero positions
    addSequential(new ResetArmEncoderCmd());
    //addSequential(new ResetWristEncoderCmd());
    //addSequential(new ResetElevatorEncoderCmd());

    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
