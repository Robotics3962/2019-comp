/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.ElevatorUpCmd;
import frc.robot.commands.ElevatorDownCmd;
import frc.robot.commands.MoveToGrabPositionCmd;
import frc.robot.commands.TalonArmPIDMove;
import frc.robot.commands.TalonWristPIDMove;
import frc.robot.commands.ElevatorDownCmd.Mode;
import frc.robot.subsystems.PIDElevator;
import frc.robot.commands.ResetArmEncoderCmd;
import frc.robot.commands.ResetWristEncoderCmd;
import frc.robot.commands.ResetElevatorEncoderCmd;
import frc.robot.commands.ShootBallCmd;
import frc.robot.commands.GrabBallCmd;
import frc.robot.commands.MoveToCarryPositionCmd;
import frc.robot.commands.MoveToGrabPositionCmd;
import frc.robot.commands.MoveToShootHighPositionCmd;
import frc.robot.commands.MoveToShootLowPositionCmd;
import frc.robot.commands.MoveToShootMiddlePositionCmd;
import frc.robot.commands.MoveToStowPositionCmd;
import frc.robot.commands.PIDArmDownCmd;
import frc.robot.commands.PIDWristDownCmd;
import frc.robot.commands.PIDArmUpCmd;
import frc.robot.commands.PIDWristUpCmd;
import frc.robot.commands.ResetAllEncoders;
import frc.robot.commands.ResetArmEncoderCmd;
import frc.robot.commands.CalibrateCmd;
import frc.robot.commands.ArmDownCmd;
import frc.robot.commands.ArmUpCmd;
import frc.robot.commands.WristUpCmd;
import frc.robot.commands.WristDownCmd;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  // get both drive and operational joysticks
  Joystick driveJoystick = new Joystick(RobotMap.Joystick0Id);
  Joystick operationJoyStick = new Joystick(RobotMap.Joystick1Id); 

  public OI() {     

    // get the buttons on the drive joystick
    JoystickButton driveButtonA = new JoystickButton(driveJoystick, RobotMap.JoystickButtonA);
    JoystickButton driveButtonB = new JoystickButton(driveJoystick, RobotMap.JoystickButtonB);
    JoystickButton driveButtonX = new JoystickButton(driveJoystick, RobotMap.JoystickButtonX);
    JoystickButton driveButtonY = new JoystickButton(driveJoystick, RobotMap.JoystickButtonY);
    JoystickButton driveButtonLS = new JoystickButton(driveJoystick, RobotMap.JoystickButtonShoulderLeft);
    JoystickButton driveButtonRS = new JoystickButton(driveJoystick, RobotMap.JoystickButtonShoulderRight);
    JoystickButton driveButtonBack = new JoystickButton(driveJoystick, RobotMap.JoystickButtonBack);
    JoystickButton driveButtonStart = new JoystickButton(driveJoystick, RobotMap.JoystickButtonStart);

    // map buttons to commands on the joystick that drives the robot
    driveButtonA.whileHeld(new ArmDownCmd());
    driveButtonB.whileHeld(new ArmUpCmd());
    driveButtonX.whileHeld(new WristDownCmd());
    driveButtonY.whileHeld(new WristUpCmd());
    driveButtonLS.whileHeld(new ElevatorDownCmd(ElevatorDownCmd.Mode.SPEED));
    driveButtonRS.whileHeld(new ElevatorUpCmd(ElevatorUpCmd.Mode.SPEED));
    driveButtonBack.whenPressed(new GrabBallCmd());
    driveButtonStart.whenPressed(new ShootBallCmd());

    // second joystick I'm calling it operational - no command mapping yet
    JoystickButton opButtonA = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonA);
    JoystickButton opButtonB = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonB);
    JoystickButton opButtonX = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonX);
    JoystickButton opButtonY = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonY);
    JoystickButton opButtonLS = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonShoulderLeft);
    JoystickButton opButtonRS = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonShoulderRight);
    JoystickButton opButtonBack = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonBack);
    JoystickButton opButtonStart = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonStart);
  }

  // these next two functions are legacy functions that get joystick values

  public double getLeftJoystickXVal() {
    double raw = driveJoystick.getX();
    return Math.abs(raw) < RobotMap.JoystickDeadZone ? 0.0 : raw;
  }

  public double getLeftJoystickYVal() {
    double raw = driveJoystick.getY();
    return Math.abs(raw) < RobotMap.JoystickDeadZone ? 0.0 : raw;
  }    

  // these next two function will allow one joystick to control
  // the thottle, and the other to control the rotation.
  // =========> it is untested <==============
  public double getLeftThrottle() {
		return driveJoystick.getY(); // Laika needs negative, Belka is positive
	}	

	public double getRightRotation() {
		return driveJoystick.getRawAxis(4);
  }
}
