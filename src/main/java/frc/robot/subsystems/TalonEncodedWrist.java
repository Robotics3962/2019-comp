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
import frc.robot.commands.WristHoldCmd;

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

  private double targetPosition;
  private double velocity;
  private boolean encodersAreEnabled = false;
  private boolean limitSwAreEnabled = false;
  private boolean manualOverride = true;
  private int count = 0;
  private int logMsgInterval = 5; // log message every N times log is called

  // set to false to use position, set to true to
  // use motion magic.  girls of steel uses motion magic for
  // what looks to be an elevator, but not for what looks
  // like a rotation
  private boolean useMotionMagic = false;

  //Limit switches;
  private DigitalInput topLimit = null; //new DigitalInput(RobotMap.LimitSwitchPIOId2);
  private DigitalInput bottomLimit = null; //new DigitalInput(RobotMap.LimitSwitchPIOId3);

  // holds variables used to determine out of phase encoders
  private Robot.Direction dirMoved = Robot.Direction.NONE; 
  private double pastPosition = 0.0;

  public TalonEncodedWrist() {
    encodersAreEnabled = true;
    limitSwAreEnabled = false;

    targetPosition = 0;
    velocity = 0;

    // assume that motor1 is connected to encoder
    
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
    motor1.setInverted(false);
    motor2.setInverted(false);

    if(limitSwAreEnabled){
      topLimit = new DigitalInput(RobotMap.WristTopLimitSwitchId);
      bottomLimit = new DigitalInput(RobotMap.WristBottomLimitSwitchId);
    }

    if (encodersAreEnabled) {
      // init code pulled from https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/MotionMagic/src/main/java/frc/robot/Robot.java

      /* Configure Sensor Source for Pirmary PID */
      /* I don't see where GOS sets this. Could it be set to something else */
      motor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,	
          PRIMARY_ENCODER_IDX, 
          ENCODER_CONFIG_TIMEOUT);

      motor1.setSensorPhase(false);

      motor1.setNeutralMode(NeutralMode.Brake);
      motor2.setNeutralMode(NeutralMode.Brake);
  
      /* Set relevant frame periods to be at least as fast as periodic rate */
      /* DJD I don't know what this does                                    */
      /* girls of steel doesn't do this                                     */
      // I don't think we want to set this, it has to do with how often position is updated,
		  //motor1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ENCODER_CONFIG_TIMEOUT);
		  //motor1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, ENCODER_CONFIG_TIMEOUT);

      /* Set Motion Magic gains in slot0 - see documentation */
      motor1.selectProfileSlot(ENCODER_SLOT_INDEX, PRIMARY_ENCODER_IDX);

      //change these parameters
      motor1.config_kF(PRIMARY_ENCODER_IDX, RobotMap.TalonWristPID_F, ENCODER_CONFIG_TIMEOUT);
      motor1.config_kP(PRIMARY_ENCODER_IDX, RobotMap.TalonWristPID_P, ENCODER_CONFIG_TIMEOUT);
      motor1.config_kI(PRIMARY_ENCODER_IDX, RobotMap.TalonWristPID_I, ENCODER_CONFIG_TIMEOUT);
      motor1.config_kD(PRIMARY_ENCODER_IDX, RobotMap.TalonWristPID_D, ENCODER_CONFIG_TIMEOUT);
  
      if(useMotionMagic){
        /* Set acceleration and vcruise velocity - see documentation */
		    motor1.configMotionCruiseVelocity(RobotMap.TalonWristCruiseSpeed, ENCODER_CONFIG_TIMEOUT);
		    motor1.configMotionAcceleration(RobotMap.TalonWristAcceleration, ENCODER_CONFIG_TIMEOUT);
      }

		  /* Zero the sensor */
      motor1.setSelectedSensorPosition(PRIMARY_ENCODER_IDX, ENCODER_RESET_POSTION, ENCODER_CONFIG_TIMEOUT);
    }

    Robot.Log("wrist is initialized");
  }

  public void setPIDPosition(double pos) {
    if(!encodersAreEnabled){
      Robot.die();
    }

    // this is an automated call so turn off manual control
    // also turn off sensor phase checking
    manualOverride = false;
    dirMoved = Direction.NONE;

    Robot.Log("wrist setPidPosition:" + pos);
    targetPosition = pos;

    // don't need to move the wrist as the background command
    // will take care of that
  }

  public void stop() {
    //motor1.neutralOutput();
    motor1.stopMotor();
  }

  public boolean resetEncoder(){
    if(!encodersAreEnabled){
      Robot.die();
    }

    stop();

    // there is no guarantee the position will be 0 when this call returns.
    // because it is done asynchronously. Have a command call encoderResetComplete()
    // until it returns true 
    motor1.setSelectedSensorPosition(PRIMARY_ENCODER_IDX, ENCODER_RESET_POSTION, ENCODER_RESET_TIMEOUT);

    // set the target position to 0 so we don't move the arm to some
    // out of bounds place
    targetPosition  = 0;

    Robot.Log("wrist talon encoders reset");
    return true;
  }

  public boolean encoderResetComplete(){
    if(!encodersAreEnabled){
      Robot.die();
    }

    boolean complete = false;
    // it may never get to 0, if the motor is moving at all
    // the encoder would return non-zero, we may have to check
    // a range
    if (getCurrentPosition() == 0){
      complete = true;
      Robot.Log("wrist Talon encoder is complete");
    }
    return complete;
  }

  public double getTargetPosition(){
    return targetPosition;
  }

  public double getCurrentPosition() {
    if(!encodersAreEnabled){
      Robot.die();
    }

    double currpos = motor1.getSelectedSensorPosition(0);
    return currpos;
  }

  public void holdPosition(){
    if(encodersAreEnabled){
      move();
    }
  }

  public void move(){
    if(!encodersAreEnabled){
      Robot.die();
    }
    // do not set the pid if manual override is enabled
    if(!manualOverride){
       // if we have reached the limits, then make sure
      // we don't move past them, so set the position we
      // want to move to to the current position
      // so we stop moving
      if(atLowerLimit() || atUpperLimit()){
        Robot.Log("Wrist is at Limits: bottom:" + atLowerLimit() + " upper:" + atUpperLimit());
        targetPosition = getCurrentPosition();
      }

      if(useMotionMagic){
        motor1.set(ControlMode.MotionMagic, targetPosition);
      }
      else {
        motor1.set(ControlMode.Position, targetPosition);
      }
      LogInfo(true);
    }
  }

  public boolean onTarget(){
    if(!encodersAreEnabled){
      Robot.die();
    }
    
    boolean reachedTarget = false;
    boolean belowRange = true;
    boolean aboveRange = true;
    double curpos = getCurrentPosition();

    belowRange = curpos < (targetPosition - RobotMap.TalonWristAbsTolerance);
    aboveRange = curpos > (targetPosition + RobotMap.TalonWristAbsTolerance);

    reachedTarget = ((!belowRange) && (!aboveRange));
    return reachedTarget;
  }

  public boolean atUpperLimit(){
    boolean atLimit = false;
    if(limitSwAreEnabled){
      atLimit = topLimit.get();
    }

    return atLimit;
  }

  public boolean atLowerLimit() {
    boolean atLimit = false;
    if(limitSwAreEnabled){
      atLimit = bottomLimit.get();
    }
    
    return atLimit;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new WristHoldCmd());
  }

  // make sure the motor and encoder are in phase.  This means that
  // when we move the motor with a negative speed, the encoder
  // show we moved in the negative direction and vice versa
  private void VerifyEncoderPhase(double prevPos){

    if(encodersAreEnabled){
      double pos = getCurrentPosition();
      double deltaPos = pos - prevPos;
      double sign = 0;
      boolean check = true;

      // don't do this check when running PID
      // as prev and curr position are not set
      // correctly and could flag a false positive
      // or a false negative
      if(!manualOverride){
        return;
      }

      switch(dirMoved){
        case DOWN:
          sign = Math.copySign(1, RobotMap.TalonWristDownSpeed);
          break;
        case UP:
          sign = Math.copySign(1, RobotMap.TalonWristUpSpeed);
          break;
        case NONE://also is referred to if using PID move
          check = false;
        break;
      }
      
      if( check && (Math.abs(deltaPos) > RobotMap.EncoderSlop) ){
        double deltaPosSign = Math.copySign(1, deltaPos);
        if( deltaPosSign != sign){
          Robot.Log("Wrist encoder is out of Phase from Wrist Motor");
          Robot.die();
        }
      }
      pastPosition = getCurrentPosition();
    }
    return;
  }

  // the rest of these commands are for manual movement

  public void setTalonSpeed(double val){
    // make sure we enable the manual override
    // which will stop the PID control from taking
    // immediately over
    manualOverride = true;
    velocity = val;
    motor1.set(ControlMode.PercentOutput, velocity);  
    LogInfo(true);
  }

  public void Up(){
    LogValues(true);
    if (atUpperLimit()){
      stop();
    }
    else {
      VerifyEncoderPhase(pastPosition);
      dirMoved = Robot.Direction.UP;
      setTalonSpeed(RobotMap.TalonWristUpSpeed);
    }
  }

  public void Down(){
    LogValues(true);
    if (atLowerLimit()){
      stop();
    }
    else {
      VerifyEncoderPhase(pastPosition);
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
    double currPos = -1;
    boolean atTarget = false;
    if(encodersAreEnabled){
      currPos = getCurrentPosition();
      atTarget = onTarget();
    }
  
    String output = "Wrist Info: manual:" + manualOverride;
    output = output + " target:" + targetPosition;
    output = output + " current:" + currPos;
    output = output + " ontarg:" + atTarget;
    output = output + " dir:" + dirMoved;
    output = output + " speed:" + velocity;
    output = output + " upLimit:" + atUpperLimit();
    output = output + " loLimit:" + atLowerLimit();
    Robot.Log(output);
  }
}
