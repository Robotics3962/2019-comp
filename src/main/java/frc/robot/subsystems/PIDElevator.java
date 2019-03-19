/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class PIDElevator extends PIDSubsystem {

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Spark motor1;
  private DigitalInput topLimit = null;
  private DigitalInput bottomLimit = null;
  private double currSpeed = 0;
  private int count = 0;
  private final int logMsgInterval = 5;

  // holds variables used to determine out of phase encoders
  private Robot.Direction dirMoved = Robot.Direction.NONE; 
  
  public PIDElevator(){
    super("PIDElevator", RobotMap.ElevatorPID_P, RobotMap.ElevatorPID_I, RobotMap.ElevatorPID_D, RobotMap.ElevatorPID_F);

    motor1 = new Spark(RobotMap.SparkElevatorId);

    // min and max speed the motor will run
    setOutputRange(RobotMap.ElevatorMinOutput, RobotMap.ElevatorMaxOutput);

    topLimit = new DigitalInput(RobotMap.ElevatorTopLimitSwitchId);
    bottomLimit = new DigitalInput(RobotMap.ElevatorBottomLimitSwitchId);

    // initialize PID Controller
    Robot.Log("PIDElevator Initialized");
  }

  @Override
  public void initDefaultCommand() {
  }

  // used to manually set the speed which will disable pid control
  public void setSpeed(double speed){
    currSpeed = speed;

    motor1.set(currSpeed);
  }

  public void Up() {
    if (atUpperLimit()){
      Stop();
    } 
    else {
      dirMoved = Robot.Direction.UP;
      setSpeed(RobotMap.ElevatorUpSpeed);
    }
  }
  
  public void Stop() {
    // or call stopmotor()?
    setSpeed(RobotMap.ElevatorStopSpeed);
  }
	
  public void Down() {
    if (atLowerLimit() || Robot.encodedArmTalon.atLowerLimit()){ //when the arm hits the liit switcht he elevator needs to be stopped as well
      Stop();
    } 
    else {
     dirMoved = Robot.Direction.DOWN;
     setSpeed(RobotMap.ElevatorDownSpeed);
    }
  }

  public boolean atUpperLimit(){
    boolean atLimit = false;
    atLimit = topLimit.get();

    return atLimit;
  }

  public boolean atLowerLimit() {
    boolean atLimit = false;
    // currently the bottom limit switched is wired
    // differently so true is not set and false is set
    // so we need to invert the logic
    atLimit = ! bottomLimit.get();

    return atLimit;
  }

  public void LogInfo(boolean dampen){
    count++;

    if(dampen && ((count % logMsgInterval) != 0)){
      return;
    }

    String output = "Elevator Info: ";
    output = output + " dir:" + dirMoved;
    output = output + " speed:" + currSpeed;
    output = output + " upLimit:" + atUpperLimit();
    output = output + " boLimit:" + atLowerLimit();
    Robot.Log(output);
  }

  protected double returnPIDInput(){
    return 0;
  }

  protected void usePIDOutput(double output){
    // do nothing
  }
}
