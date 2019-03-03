/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.TalonEncodedArm;
import frc.robot.subsystems.TalonEncodedWrist;
import frc.robot.subsystems.PIDElevator;
import frc.robot.subsystems.DiffDriveBase;
import frc.robot.subsystems.Intake;
import frc.robot.utils.CollectorPosition;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // subsystems
  public static TalonEncodedArm encodedArmTalon = null;
  public static TalonEncodedWrist encodedWristTalon = null;
  public static PIDElevator pidElevator = null;
  public static DiffDriveBase diffDriveBase = null;
  public static Intake intake = null;

  // this array holds the position of the collector in each index
  public static CollectorPosition[] collectorPositions;

  // copy the constants into the array to make access cleaner
  public static void initPositionArray(){
    collectorPositions = new CollectorPosition[RobotMap.MaxBallPosIndex];
    collectorPositions[RobotMap.GrabBallPosIndex] = new CollectorPosition(RobotMap.GrabBallElevatorPos, RobotMap.GrabBallArmPos, RobotMap.GrabBallWristPos);
    collectorPositions[RobotMap.StowPosIndex] = new CollectorPosition(RobotMap.StowBallElevatorPos, RobotMap.StowBallArmPos, RobotMap.StowBallWristPos);
    collectorPositions[RobotMap.LowBallPosIndex] = new CollectorPosition(RobotMap.LowBallElevatorPos, RobotMap.LowBallArmPos, RobotMap.LowBallWristPos);
    collectorPositions[RobotMap.MiddleBallPosIndex] = new CollectorPosition(RobotMap.MiddleBallElevatorPos, RobotMap.MiddleBallArmPos, RobotMap.MiddleBallWristPos);
    collectorPositions[RobotMap.HighBallPosIndex] = new CollectorPosition(RobotMap.HighBallElevatorPos, RobotMap.HighBallArmPos, RobotMap.HighBallWristPos);
    collectorPositions[RobotMap.CarryBallPosIndex] = new CollectorPosition(RobotMap.CarryBallElevatorPos, RobotMap.CarryBallArmPos, RobotMap.CarryBallWristPos);
  }

  // this is used to log output to the console
  public static void Log(String msg){
    System.out.println(msg);
  }
  
  // used in out of phase encoder detection
  public enum Direction {
    NONE,UP,DOWN
  }

  // this is a divide by 0 which will 
  // throw an exception which should 
  // stop the program from running or otherwise
  // indicate an error
  public static void die(){
    int x = 0;
    int u = 1/x;
  }

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    // make the positions to move the collector into
    // easy access
    initPositionArray();

    // create all subsystems
    diffDriveBase = new DiffDriveBase();
    encodedWristTalon = new TalonEncodedWrist();
    encodedArmTalon = new TalonEncodedArm();
    pidElevator = new PIDElevator();
    intake = new Intake();
    
    // call control loop
    m_oi = new OI();

    //m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    //SmartDashboard.putData("Auto mode", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
