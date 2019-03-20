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
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; 
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Add your docs here.
 */
public class TalonEncodedWrist extends Subsystem {

  private enum Mode { NONE, SPEED, PID };

  // Put methods for controlling this subsystem
  // here. Call these from Commands. 
  private static final int ENCODER_SLOT_INDEX = 0;
  private static final int PRIMARY_ENCODER_IDX = 0;
  private static final int ENCODER_RESET_POSTION = 0;
  private static final int ENCODER_CONFIG_TIMEOUT = 10;
  private static final int TALONRSX_TIMEOUT = 10;
  private static final int PID_TARGET_ADJUST_STEPS = 2;

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

  // used for locking the position of the wrist
  private double targetPosition = 0;
  private double newTargetPosition = 0;
  private Mode mode = Mode.SPEED;
  private boolean useEncoder = false;

  // set to true if limit switches set wired
  private boolean useLimitSw = false;

  /**
   * This is the subsystem the controls the wrist. 
   * 
   * It has optional support for limit switches.  If limit
   * switches are present, set useLimitSw to true.  
   * 
   * It has optional support for an encoder.  If an encoder
   * is present set useEncoder to true.
   * 
   * Its primary responsibility is to allow manual control of the wrist.
   * A thumbstick on the game pad is used to set the speed in
   * a positive or negative direction.  Postitive rotates the
   * wrist up, negative rotates the wrist down.  
   * 
   * The speed the wrist moves is dependent on how far the joystick
   * is moved.  The entire movement of the joystick is scaled to 
   * constants set in RobotMap.java.  This allows finer control and
   * stops the wrist from moving too fast.
   * 
   * If an encode is present, the wrist can be locked into its current
   * postion by calling LockWristPosition().  This uses the pid 
   * controller to keep the wrist at the current position. 
   * 
   * The locked position can be moved up and down for fine adjustments.
   */
  public TalonEncodedWrist() {
    velocity = 0;

    motor1 = new WPI_TalonSRX(RobotMap.TalonMotorCanID3);
    motor2 = new WPI_TalonSRX(RobotMap.TalonMotorCanID4);

    motor1.configFactoryDefault();
    motor2.configFactoryDefault();
  
    // http://www.ctr-electronics.com/downloads/api/java/html/enumcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1_neutral_mode.html#a3128d32cfbfab8d5e40ed6907d14e621
    //When commanded to neutral, motor leads are commonized electrically to reduce motion. 
    motor1.setNeutralMode(NeutralMode.Brake);
    motor2.setNeutralMode(NeutralMode.Brake);

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

    if(useLimitSw){
      topLimit = new DigitalInput(RobotMap.WristTopLimitSwitchId);
      bottomLimit = new DigitalInput(RobotMap.WristBottomLimitSwitchId);
    }
    if (useEncoder) {
      // init code pulled from https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/MotionMagic/src/main/java/frc/robot/Robot.java

      /* Configure Sensor Source for Pirmary PID */
      /* GOS does not call this function, I guess they take the default */
      /* they could be using the absolute location mode */
      motor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,	
          PRIMARY_ENCODER_IDX, 
          ENCODER_CONFIG_TIMEOUT);

      motor1.setSensorPhase(true);

      motor1.selectProfileSlot(ENCODER_SLOT_INDEX, PRIMARY_ENCODER_IDX);

      motor1.config_kF(0, RobotMap.TalonWristPID_F, ENCODER_CONFIG_TIMEOUT);
      motor1.config_kP(0, RobotMap.TalonWristPID_P, ENCODER_CONFIG_TIMEOUT);
      motor1.config_kI(0, RobotMap.TalonWristPID_I, ENCODER_CONFIG_TIMEOUT);
      motor1.config_kD(0, RobotMap.TalonWristPID_D, ENCODER_CONFIG_TIMEOUT);
  
		  /* Zero the sensor */
      motor1.setSelectedSensorPosition(PRIMARY_ENCODER_IDX, ENCODER_RESET_POSTION, ENCODER_CONFIG_TIMEOUT);
    }

    Robot.Log("wrist is initialized");
  }

  // this stops the motor from moving. With the default neutral
  // position set to brake, it should stop it from moving.
  public void stop() {
    //motor1.neutralOutput();
    motor1.stopMotor();
  }

  /**
   * This function returns if the upper limit switch is
   * engaged. 
   * 
   * @return true: if the upper limit switch is activated
   * if limit switches are not configured, false is always returned
   */
  public boolean atUpperLimit(){
    boolean atLimit = false;
    if(useLimitSw){
      atLimit = topLimit.get();
    }
    return atLimit;
  }

  /**
   * This function returns if the lower limit switch is
   * engaged. 
   * 
   * @return true: if the lower limit switch is activated
   * if limit switches are not configured, false is always returned
   */
  public boolean atLowerLimit() {
    boolean atLimit = false;
    if(useLimitSw){
      atLimit = bottomLimit.get();
    }
    return atLimit;
  }

  @Override
  public void initDefaultCommand() {
  }

  /**
   * gets the current position of the encoder
   * 
   * @return a double with the current encoder position
   * if no encoder is configured, it returns 0.0
   */
  private double getCurrentPosition(){
    double pos = 0.0;
    if(useEncoder){
      pos = motor1.getSelectedSensorPosition(0);
    }
    return pos;
  }

  /**
   * tells the subsystem to use the pid controller to
   * keep the wrist locked at its current position
   */
  public void LockPosition(){
    if(useEncoder){
      mode = Mode.PID;
      newTargetPosition = getCurrentPosition();
    }
  }

  /**
   * tells teh subsystem to no longer keep the 
   * wrist locked at its position and the wrist can
   * now be moved by using the thumbstick to change the speed.
   */
  public void UnlockPosition(){
    mode = Mode.SPEED;
  }

  /**
   * adjustes the target position the wrist is locked at
   * in the upward direction by a fixed amount
   */
  public void MoveTargetPositionUp(){
    dirMoved = Robot.Direction.UP;
    if(!atUpperLimit() && useEncoder){
      newTargetPosition += PID_TARGET_ADJUST_STEPS;
    }
  }

  /**
   * adjustes the target position the wrist is locked at
   * in the downward direction by a fixed amount
   */
  public void MoveTargetPositionDown(){
    dirMoved = Robot.Direction.DOWN;
    if(!atLowerLimit() && useEncoder){
      newTargetPosition -= PID_TARGET_ADJUST_STEPS;
    }
  }

  // the rest of these commands are for manual movement

  /**
   * set the speed to move the motors at
   * @param val
   * this is the speed to move the motor, it will be
   * between -1 and +1
   */
  public void setTalonSpeed(double val){
    velocity = val;
    motor1.set(ControlMode.PercentOutput, velocity);  
  }

  /**
   * this logs information about the wrist 
   * @param dampen
   * if dampen is true, only one out of logMsgInterval
   * messages is displayed.  Displaying all messages can
   * take up too much cpu and overrun the console buffer
   */
  public void LogInfo(boolean dampen){
    count++;

    if(dampen && ((count % logMsgInterval) != 0)){
      return;
    }

    String output = "Wrist Info: ";
    if(useEncoder){
      output = output + " target:" + targetPosition;
      output = output + " curpos:" + getCurrentPosition();
    }
    output = output + " dir:" + dirMoved;
    output = output + " speed:" + velocity;
    output = output + " upLimit:" + atUpperLimit();
    output = output + " loimit:" + atLowerLimit();
    Robot.Log(output);     
  }

  /**
   * This functio does all of the work.  It is called before
   * any of the commands are executed.  It works in PID mode whic
   * is used to lock the wrist at a certain position.  Or it uses
   * speed mode which determines the speed to move the motor by
   * looking at values returned by a thumb stick 
   */
  public void periodic(){

    // we are in pid mode when we lock the wrist.
    if(mode == Mode.PID && useEncoder){
      targetPosition = newTargetPosition;
      motor1.set(ControlMode.Position, targetPosition);
    }
    else if(mode == Mode.SPEED){
      double speed = Robot.m_oi.getOperWristControl() * -1;
      double scaledSpeed;
        
      scaledSpeed = speed * RobotMap.WristScaledSpeedFactor;

      if(scaledSpeed > 0){
        if (atUpperLimit()){
          stop();
        }
        
        dirMoved = Robot.Direction.UP;
      }
      else if(scaledSpeed < 0){
        if (atLowerLimit()){
          stop();
        }
        
        dirMoved = Robot.Direction.DOWN;
      }
      else {
        dirMoved = Robot.Direction.NONE;
      }

      setTalonSpeed(scaledSpeed);
    }
  }
}
