/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Robot.Direction;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; 
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import java.lang.Math;

/**
 * Add your docs here.
 */
public class TalonEncodedWrist extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands. 
  private static final int ENCODER_SLOT_INDEX = 0;
  private static final int PRIMARY_ENCODER_IDX = 0;
  private static final int ENCODER_RESET_POSTION = 0;
  private static final int ENCODER_RESET_TIMEOUT = 0;
  private static final int ENCODER_CONFIG_TIMEOUT = 10;
  private static final int TALONRSX_TIMEOUT = 10;

  private WPI_TalonSRX motor1;
  private WPI_TalonSRX motor2;

  private double velocity;
  private int count = 0;
  private int logMsgInterval = 5; // log message every N times log is called

  //Limit switches;
  private DigitalInput topLimit = null; //new DigitalInput(RobotMap.LimitSwitchPIOId2);
  private DigitalInput bottomLimit = null; //new DigitalInput(RobotMap.LimitSwitchPIOId3);

  // holds variables used to determine out of phase encoders
  private Robot.Direction dirMoved = Robot.Direction.NONE; 

  public TalonEncodedWrist() {
    velocity = 0;

    motor1 = new WPI_TalonSRX(RobotMap.TalonMotorCanID3);
    motor2 = new WPI_TalonSRX(RobotMap.TalonMotorCanID4);

    motor1.configFactoryDefault();
    motor2.configFactoryDefault();

    // only 1 controller (motor1) is wired to the encoder, so we have motor2
    // follow motor1 to keep it moving at the same speed
    motor2.follow(motor1);
    
		/* Set the peak and nominal outputs */
		motor1.configNominalOutputForward(0, TALONRSX_TIMEOUT);
		motor1.configNominalOutputReverse(0, TALONRSX_TIMEOUT);
		motor1.configPeakOutputForward(RobotMap.TalonMaxOutput, TALONRSX_TIMEOUT);
    motor1.configPeakOutputReverse(RobotMap.TalonMinOutput, TALONRSX_TIMEOUT);
    
    // this could be either true or false, we have to determine
    // how it is confgured
    motor1.setInverted(true);
    motor2.setInverted(true);

    topLimit = new DigitalInput(RobotMap.WristTopLimitSwitchId);
    bottomLimit = new DigitalInput(RobotMap.WristBottomLimitSwitchId);

    Robot.Log("wrist is initialized");
  }

  public void stop() {
    //motor1.neutralOutput();
    motor1.stopMotor();
  }


  public boolean atUpperLimit(){
    boolean atLimit = false;
    atLimit = topLimit.get();

    return atLimit;
  }

  public boolean atLowerLimit() {
    boolean atLimit = false;
    atLimit = bottomLimit.get();
    
    return atLimit;
  }

  @Override
  public void initDefaultCommand() {
  }

  // the rest of these commands are for manual movement

  public void setTalonSpeed(double val){
    velocity = val;
    motor1.set(ControlMode.PercentOutput, velocity);  
  }

  public void Up(){
    LogInfo(true);
    if (atUpperLimit()){
      stop();
    }
    else {
      dirMoved = Robot.Direction.UP;
      setTalonSpeed(RobotMap.TalonWristUpSpeed);
    }
  }

  public void Down(){
    LogInfo(true);
    if (atLowerLimit()){
      stop();
    }
    else {
      dirMoved = Robot.Direction.DOWN;
      setTalonSpeed(RobotMap.TalonWristDownSpeed);
    }
  }

  public void Stop(){
    dirMoved = Direction.NONE;
    // or call motor1.stopMotor();
    //setTalonSpeed(RobotMap.TalonWristStopSpeed);
    stop();
  }

  public void LogInfo(boolean dampen){
    count++;

    if(dampen && ((count % logMsgInterval) != 0)){
      return;
    }

    String output = "Wrist Info: ";
    output = output + " dir:" + dirMoved;
    output = output + " speed:" + velocity;
    output = output + " upLimit:" + atUpperLimit();
    output = output + " loimit:" + atLowerLimit();
    Robot.Log(output);     
  }

  public void periodic(){
    double speed = Robot.m_oi.getOperWristControl() * -1;
    double scaledSpeed;
      
    scaledSpeed = speed * RobotMap.WristScaledSpeedFactor;

    if(scaledSpeed > 0){
      dirMoved = Robot.Direction.UP;
    }
    else if(scaledSpeed < 0){
      dirMoved = Robot.Direction.DOWN;
    }
    else {
      dirMoved = Robot.Direction.NONE;
    }

    setTalonSpeed(scaledSpeed);
  }
}
