/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {

	private Spark motor;
	private double currSpeed;
	public enum Operation { STOP, SHOOT, GRAB};
	private Operation op = Operation.STOP;
    public Intake(){
		motor = new Spark(RobotMap.SparkIntakeId);
		motor.enableDeadbandElimination(true);	
	}
		
	public void grabIntake() {
		op = Operation.GRAB;
		currSpeed = RobotMap.IntakeIngressSpeed;
		motor.set(currSpeed);
		LogInfo();
	}
	
	public void shootIntake() {
		op = Operation.SHOOT;
		currSpeed = RobotMap.IntakeEgressSpeed;
		motor.set(currSpeed);
		LogInfo();
	}
	
	public void stopIntake() {
		op = Operation.STOP;		
		currSpeed = RobotMap.IntakeStopSpeed;
		motor.stopMotor();
		LogInfo();
    }

    @Override
    protected void initDefaultCommand() {
	}
	
	public void LogInfo(){
		Robot.UpdateDashboard("intake.speed", currSpeed);
		String tmp = ""+ op;
		Robot.UpdateDashboard("intake.op", tmp);
	}
}
