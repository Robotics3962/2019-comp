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
    driveButtonA.whenPressed(new MoveToGrabPositionCmd());
    driveButtonB.whenPressed(new MoveToShootLowPositionCmd());
    driveButtonX.whenPressed(new MoveToShootMiddlePositionCmd());
    driveButtonY.whenPressed(new MoveToShootHighPositionCmd());
    driveButtonBack.whenPressed(new MoveToStowPositionCmd());
    driveButtonStart.whenPressed(new MoveToCarryPositionCmd());
    driveButtonLS.whileHeld(new ShootBallCmd());
    driveButtonRS.whileHeld(new GrabBallCmd());

    // get the buttons on the operational joystick
    JoystickButton opButtonA = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonA);
    JoystickButton opButtonB = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonB);
    JoystickButton opButtonX = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonX);
    JoystickButton opButtonY = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonY);
    JoystickButton opButtonLS = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonShoulderLeft);
    JoystickButton opButtonRS = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonShoulderRight);
    JoystickButton opButtonBack = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonBack);
    JoystickButton opButtonStart = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonStart);

    // map buttons to commands on the has manual adjustments
    opButtonA.whileHeld(new PIDArmDownCmd());
    opButtonB.whileHeld(new PIDArmUpCmd());
    opButtonX.whileHeld(new PIDWristDownCmd());
    opButtonY.whileHeld(new PIDWristUpCmd());
    opButtonLS.whileHeld(new ElevatorDownCmd());
    opButtonRS.whileHeld(new ElevatorUpCmd());

    // this should be used only under extreme duress.
    // will zero the positions of the elevator,arm, and wrist
    // where they currently are. If the encoders get really out
    // of sync, hit this button, then use A|B|X|Y|LS|RS to move
    // the components to the positions they were originally zeroed
    // at them hit this button so the movetoposition buttons
    // above will move to the correct positions
    opButtonBack.whenPressed(new ResetAllEncoders());
  }

  // these next to functions are legacy functions that get joystick values

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
