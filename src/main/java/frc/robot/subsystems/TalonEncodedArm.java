/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.Robot.Direction;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; 
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;

public class TalonEncodedArm extends Subsystem {

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
  private int logMsgInterval = 50;
  
  // holds variables used to determine out of phase encoders
  private Robot.Direction dirMoved = Robot.Direction.NONE; 

  //Limit switches;
  private DigitalInput topLimit = null; //new DigitalInput(RobotMap.LimitSwitchPIOId2);
  private DigitalInput bottomLimit = null; //new DigitalInput(RobotMap.LimitSwitchPIOId3);

  public TalonEncodedArm() {

    velocity = 0;

    // assume that motor1 is connected to encoder
    motor1 = new WPI_TalonSRX(RobotMap.TalonMotorCanId1);
    motor2= new WPI_TalonSRX(RobotMap.TalonMotorCanId2); 
   
    motor1.configFactoryDefault();
    motor2.configFactoryDefault();

    // only 1 controller (motor1) is wired to the encoder, so we have motor2
    // follow motor1 to keep it moving at the same speed
    motor2.follow(motor1);
    
    /* Set the peak and nominal outputs */
    /* gos doesn't call these functions */
		motor1.configNominalOutputForward(0, TALONRSX_TIMEOUT);
		motor1.configNominalOutputReverse(0, TALONRSX_TIMEOUT);
		motor1.configPeakOutputForward(RobotMap.TalonArmMaxOutput, TALONRSX_TIMEOUT);
    motor1.configPeakOutputReverse(RobotMap.TalonArmMinOutput, TALONRSX_TIMEOUT);

    // this could be either true or false, we have to determine
    // how it is confgured
    motor1.setInverted(false);
    motor2.setInverted(false);

    topLimit = new DigitalInput(RobotMap.ArmTopLimitSwitchId);
    bottomLimit = new DigitalInput(RobotMap.ArmBottomLimitSwitchId);


    motor1.setNeutralMode(NeutralMode.Brake);
    motor2.setNeutralMode(NeutralMode.Brake);

    Robot.Log("Arm Talon is initialized");
  }

  public void stop() {
    //motor1.neutralOutput();
    motor1.stopMotor();
  }

  public void initDefaultCommand(){
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


  // these commands are used to manually move the arm
  // outside of the PID control

  public void setTalonSpeed(double val){

    velocity = val;
    motor1.set(ControlMode.PercentOutput, velocity);
  }

  public void Up(){
    LogInfo(true);  
    if (atUpperLimit()){
      Stop();
    }
    else {
      dirMoved = Robot.Direction.UP;
      setTalonSpeed(RobotMap.TalonArmUpSpeed);
    }
  }

  public void Down(){
    LogInfo(true);  
    if (atLowerLimit()){
      Stop();
    }
    else {
      dirMoved = Robot.Direction.DOWN;
      setTalonSpeed(RobotMap.TalonArmDownSpeed);
    }
  }

  public void Stop(){
    dirMoved = Direction.NONE;
    // or call motor1.stopMotor()
    stop();
  }

  public void LogInfo(boolean dampen){
    count++;
    
    if(dampen && ((count % logMsgInterval) != 0)){
      return;
    }
  
    String output = "Arm Info: ";
    output = output + " dir:" + dirMoved;
    output = output + " speed:" + velocity;
    output = output + " upLimit:" + atUpperLimit();
    output = output + " boLimit:" + atLowerLimit();
    Robot.Log(output);

  }

  public void periodic(){
    double speed = Robot.m_oi.getOperArmControl() * -1;
    double scaledSpeed;
      
    scaledSpeed = speed * RobotMap.ArmScaledSpeedFactor;

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

