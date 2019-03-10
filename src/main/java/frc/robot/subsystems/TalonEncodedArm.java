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
import frc.robot.commands.ArmHoldCmd;

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

public class TalonEncodedArm extends Subsystem {

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
  private boolean encodersAreEnabled = true;
  private boolean ArmlimitsAreEnabled = true; //used to be private
  private boolean manualOverride = false;
  private int count = 0;
  private int logMsgInterval = 50;
  private double prevPosition = 0;
  
  // set to false to use position, set to true to
  // use motion magic.  girls of steel uses motion magic for
  // what looks to be an elevator, but not for what looks
  // like a rotation
  private boolean useMotionMagic = false;

  // holds variables used to determine out of phase encoders
  private Robot.Direction dirMoved = Robot.Direction.NONE; 
  private Robot.Direction pidDirMoved = Robot.Direction.NONE; 
  private double pastPosition = 0.0;

  //Limit switches;
  private DigitalInput topLimit = null; //new DigitalInput(RobotMap.LimitSwitchPIOId2);
  private DigitalInput bottomLimit = null; //new DigitalInput(RobotMap.LimitSwitchPIOId3);

  public TalonEncodedArm() {

    targetPosition = 0;
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

    if(ArmlimitsAreEnabled){
      topLimit = new DigitalInput(RobotMap.ArmTopLimitSwitchId);
      bottomLimit = new DigitalInput(RobotMap.ArmBottomLimitSwitchId);
    }

    if (encodersAreEnabled) {
      // init code pulled from https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/MotionMagic/src/main/java/frc/robot/Robot.java

      /* Configure Sensor Source for Pirmary PID */
      /* GOS does not call this function, I guess they take the default */
      /* they could be using the absolute location mode */
      motor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,	
          PRIMARY_ENCODER_IDX, 
          ENCODER_CONFIG_TIMEOUT);

      motor1.setSensorPhase(true);

      /* Set relevant frame periods to be at least as fast as periodic rate */
      /* DJD I don't know what this does                                    */
      /* girls of steel doesn't do this                                     */
		  //motor1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ENCODER_CONFIG_TIMEOUT);
		  //motor1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, ENCODER_CONFIG_TIMEOUT);

      /* Set Motion Magic gains in slot0 - see documentation */
      /* girls of steel uses motion magic for elevator, but position */
      /* for the pivot, so copy them                         */
      motor1.selectProfileSlot(ENCODER_SLOT_INDEX, PRIMARY_ENCODER_IDX);

      motor1.config_kF(0, RobotMap.TalonArmPID_F, ENCODER_CONFIG_TIMEOUT);
      motor1.config_kP(0, RobotMap.TalonArmPID_P, ENCODER_CONFIG_TIMEOUT);
      motor1.config_kI(0, RobotMap.TalonArmPID_I, ENCODER_CONFIG_TIMEOUT);
      motor1.config_kD(0, RobotMap.TalonArmPID_D, ENCODER_CONFIG_TIMEOUT);
  
      if(useMotionMagic){
  		  /* Set acceleration and vcruise velocity - see documentation */
		    motor1.configMotionCruiseVelocity(RobotMap.TalonArmCruiseSpeed, ENCODER_CONFIG_TIMEOUT);
        motor1.configMotionAcceleration(RobotMap.TalonArmAcceleration, ENCODER_CONFIG_TIMEOUT);
      }

		  /* Zero the sensor */
      motor1.setSelectedSensorPosition(PRIMARY_ENCODER_IDX, ENCODER_RESET_POSTION, ENCODER_CONFIG_TIMEOUT);
    }

    motor1.setNeutralMode(NeutralMode.Brake);
    motor2.setNeutralMode(NeutralMode.Brake);

    Robot.Log("Arm Talon is initialized");
  }

  public void setPIDPosition(double pos) {
    if(!encodersAreEnabled){
      return;
    }

    // need different P factors for down vs up
    // so we keep track of direction, if we move
    // from up to down we set the down pid, otherwise
    // we set the up pid.  the up p is way too much for
    // the down direction, the alernative is to somehow
    // limit the down voltage.

    if((pos>=0) && (prevPosition < 0)){
      // if i'm moving in a positive direction and the
      // previous direction was negative, then change the
      // p value to the up p value
      Robot.Log("Changing P to UP");
      pidDirMoved = Direction.UP;
      motor1.config_kP(0, RobotMap.TalonArmPID_P, ENCODER_CONFIG_TIMEOUT);      
      CleanUpPositionsAndLimits(Direction.UP);     
    }
    else if((pos < 0) && (prevPosition >= 0)){
      // if i'm moving in a negative direction and the
      // previous direction was positive, then change the
      // p value to the down p value
      Robot.Log("change P to down");
      pidDirMoved = Direction.DOWN;
      motor1.config_kP(0, RobotMap.TalonArmPIDDown_P, ENCODER_CONFIG_TIMEOUT); 
      CleanUpPositionsAndLimits(Direction.DOWN);     
    }

    double delta = getCurrentPosition() + pos;

    // this is called for PID control so turn off
    // manual override, also turn off phase
    // checking
    //Robot.die();
    manualOverride = false;
    dirMoved = Direction.NONE;

    Robot.Log("Arm setPidPosition:" + pos);
    targetPosition = AdjustTargetPositionForLimits(pos);

    // don't need to move the arm as the background command
    // will take care of that

    // keep track of the position being moved to
    // so we can detect direction change
    prevPosition = pos;
  }

  public void stop() {
    //motor1.neutralOutput();
    motor1.stopMotor();
  }

  public boolean resetEncoder(){
    if(!encodersAreEnabled){
      return true; 
      //Robot.die();
    }

    // stop the motor as we shouldn't reset when moving
    stop();

    // there is no guarantee the position will be 0 when this call returns.
    // because it is done asynchronously. Have a command call encoderResetComplete()
    // until it returns true 
    motor1.setSelectedSensorPosition(PRIMARY_ENCODER_IDX, ENCODER_RESET_POSTION, ENCODER_RESET_TIMEOUT);

    // set the target position to 0 so we don't move the arm to some
    // out of bounds place
    targetPosition  = 0;

    Robot.Log("Arm talon encoders reset");
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
      Robot.Log("Arm Talon encoder reset is complete");
    }
    else {
      Robot.Log("Arm Talon encoder reset is not complete");
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

    // only move via PID when we are not manually controlling
    if(!manualOverride){
      CheckForOverruns();

      if(useMotionMagic){
        motor1.set(ControlMode.MotionMagic, targetPosition);
      }
      else {
        motor1.set(ControlMode.Position, targetPosition);
      }
      // this takes care of the situation where the current pos
      // passes the targetpos and a limit switch is hit in the time
      // between when we checked before we moved the motor
      CheckForOverruns();
      LogInfo(true);
    }
  }
  
  public void CheckForOverruns(){
    double newTargetPosition;
    newTargetPosition = AdjustTargetPositionForLimits(targetPosition);
    if(newTargetPosition != targetPosition){
      targetPosition = newTargetPosition;
      Robot.Log("Adjusting targetposition2");
    }
  }

  private double AdjustTargetPositionForLimits(double target){
      double adjustedPos = target;

      // if we have reached the limits, then make sure
      // we don't move past them, so set the position we
      // want to move to to the current position
      // so we stop moving
      if(target > getCurrentPosition() && atUpperLimit()){
        adjustedPos = getCurrentPosition();
        Robot.Log("target Pos > current pos, setting target from " + target + " to " + adjustedPos);
      }
      
      if(target < getCurrentPosition() && atLowerLimit()){
        adjustedPos = getCurrentPosition();
        Robot.Log("target Pos < current pos, setting target from " + target + " to " + adjustedPos);
      }

      return adjustedPos;
  }

  private void CleanUpPositionsAndLimits(Direction dir){
    double adjustedPos;
    double currPos = getCurrentPosition();
    switch(dir){
      case UP:
        if(atUpperLimit() && (currPos > targetPosition)){
          targetPosition = currPos;
        }
        break;
      case DOWN:
      if(atLowerLimit() && (currPos < targetPosition)){
        targetPosition = currPos;
      }
      break;
      case NONE:
        break;
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

    belowRange = curpos < (targetPosition - RobotMap.TalonArmAbsTolerance);
    aboveRange = curpos > (targetPosition + RobotMap.TalonArmAbsTolerance);

    reachedTarget = ((!belowRange) && (!aboveRange));
    return reachedTarget;
  }

  public void initDefaultCommand(){
    if(encodersAreEnabled){
      setDefaultCommand(new ArmHoldCmd());
    }
  }

  public boolean atUpperLimit(){
    boolean atLimit = false;
    if(ArmlimitsAreEnabled){
      atLimit = topLimit.get();
    }

    if(atLimit & (targetPosition > getCurrentPosition())){
      Robot.Log("at upper limit adjust from " + targetPosition + " to " + getCurrentPosition());
      targetPosition = getCurrentPosition();
    }

    return atLimit;
  }

  public boolean atLowerLimit() {
    boolean atLimit = false;
    if(ArmlimitsAreEnabled){
      atLimit = bottomLimit.get();
    }
    
    if(atLimit && (targetPosition < getCurrentPosition())){
      Robot.Log("at lower limit adjust from " + targetPosition + " to " + getCurrentPosition());
      targetPosition = getCurrentPosition();
    }

    return atLimit;
  }

  // make sure the motor and encoder are in phase.  This means that
  // when we move the motor with a negative speed, the encoder
  // show we moved in the negative direction and vice versa
  private void VerifyEncoderPhase(double prevPos){
    // don't do this check when running PID
    // as prev and curr position are not set
    // correctly and could flag a false positive
    // or a false negative
    if(!manualOverride || !encodersAreEnabled){
      return;
    }

    double pos = getCurrentPosition();
    double deltaPos = pos - prevPos;
    double sign = 0;
    boolean check = true;

    switch(dirMoved){//direction moved
      case DOWN:
        sign = Math.copySign(1, RobotMap.TalonArmDownSpeed);
        break;
      case UP:
        sign = Math.copySign(1, RobotMap.TalonArmUpSpeed);
        break;
      case NONE:
        check = false;
      break;
    }

    if( check && (Math.abs(deltaPos) > RobotMap.EncoderSlop) ){
      double deltaPosSign = Math.copySign(1, deltaPos);
      if( deltaPosSign != sign){
        Robot.Log("Arm encoder is out of Phase from Arm Motor dir:" + dirMoved + " deltapos:" + deltaPos);
        Robot.die();
      }
    }
    pastPosition = getCurrentPosition();
    return;
  }

  // these commands are used to manually move the arm
  // outside of the PID control

  public void setTalonSpeed(double val){

    targetPosition = getCurrentPosition();
    
    // this command is used to manually move the
    // motor so set the variable that stops the PID
    // from overriding
    Robot.die(); 
    manualOverride = true;
    velocity = val;
    motor1.set(ControlMode.PercentOutput, velocity);
  }

  public void Up(){
    // stop the pid loop from moving the motor
    targetPosition = getCurrentPosition();

    LogInfo(true);  
    if (atUpperLimit()){
      Stop();
    }
    else {
      VerifyEncoderPhase(pastPosition);
      // do not set this before verify
      dirMoved = Robot.Direction.UP;
      setTalonSpeed(RobotMap.TalonArmUpSpeed);
    }
  }

  public void Down(){
    // stop the pid loop from moving the motor
    targetPosition = getCurrentPosition();
    LogInfo(true);  
    if (atLowerLimit()){
      Stop();
    }
    else {
      VerifyEncoderPhase(pastPosition);
      // do not set this before verify
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

    double currPos = -1;
    boolean atTarget = false;
    if(encodersAreEnabled){
      currPos = getCurrentPosition();
      atTarget = onTarget();
    }
  
    String output = "Arm Info: manual:" + manualOverride;
    output = output + " target:" + targetPosition;
    output = output + " current:" + currPos;
    output = output + " ontarg:" + atTarget;
    output = output + " dir:" + dirMoved;
    output = output + " speed:" + velocity;
    output = output + " upLimit:" + atUpperLimit();
    output = output + " boLimit:" + atLowerLimit();
    Robot.Log(output);

    //Robot.UpdateDashboard("Arm.manual", manualOverride); 
    //Robot.UpdateDashboard("Arm.targetPos", targetPosition); 
    //Robot.UpdateDashboard("Arm.currPos", currPos);
    //Robot.UpdateDashboard("Arm.speed", velocity);
    //Robot.UpdateDashboard("Arm.upLimit", atUpperLimit());
    //Robot.UpdateDashboard("Arm.loLimit", atLowerLimit());
  }
}

