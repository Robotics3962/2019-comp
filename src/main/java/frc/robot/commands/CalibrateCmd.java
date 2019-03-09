/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.ElevatorPIDMoveCmd;
import frc.robot.commands.ResetElevatorEncoderCmd;
import frc.robot.commands.TalonArmPIDMove;
import frc.robot.commands.TalonWristPIDMove;
import frc.robot.commands.ResetArmEncoderCmd;
import frc.robot.commands.ResetWristEncoderCmd;
import frc.robot.RobotMap;
import frc.robot.Robot;

public class CalibrateCmd extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CalibrateCmd() {
    requires(Robot.pidElevator);
    requires(Robot.encodedArmTalon);
    requires(Robot.encodedWristTalon);

    double currPos;
    double downDistance;
    double target;
    boolean ArmHasLimitSwitches = true;
    boolean WristHasLowerLimitSwitch = false;
    boolean ElevatorHasLimitSwitches = false;

    // calibrate elevator by moving up, then down to hit limit switch
    // then reset the encoder
    // down should be a large number, it is ok because we want to 
    // make sure we hit the lower limit switch
    if(ElevatorHasLimitSwitches) {
      Robot.Log("Elevator calibration start");
      currPos = Robot.pidElevator.getCurrentPosition();
      downDistance = RobotMap.ElevatorCalibrateDownDist;
      target = currPos + RobotMap.ElevatorCalibrateUpDist;
      addSequential(new ElevatorPIDMoveCmd(target));
      addSequential(new ElevatorPIDMoveCmd(downDistance));
      addSequential(new ResetElevatorEncoderCmd());
      Robot.Log("Elevator calibration completed");
    }

    if(ArmHasLimitSwitches){
      // this assumes there is a lower limit switch on the arm
      // and that the arm position is near it
      addSequential(new TalonArmPIDMove(RobotMap.TalonArmCalibrateUpDist));
      addSequential(new TalonArmPIDMove(RobotMap.TalonArmCalibrateDownDist));
      //addSequential(new ResetArmEncoderCmd());
    }
    else {
      addSequential(new ResetArmEncoderCmd());
    }
  
    Robot.Log("Wrist calibration start");
    if(WristHasLowerLimitSwitch){
      // this assumes there is a lower limit switch on the arm
      // and that the arm position is near it
      currPos = Robot.encodedWristTalon.getCurrentPosition();
      target = currPos + RobotMap.TalonWristCalibrateUpDist;
      addSequential(new TalonWristPIDMove(target));
      addSequential(new TalonWristPIDMove(RobotMap.TalonArmCalibrateDownDist));
      addSequential(new ResetWristEncoderCmd());
      Robot.Log("Wrist calibration completed");
    }
    else {
      addSequential(new ResetWristEncoderCmd());
    }
    Robot.Log("Arm calibration completed");

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
