/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {

    Spark motor;

    public Intake(){
		motor = new Spark(RobotMap.SparkIntakeId);
		motor.enableDeadbandElimination(true);	
	}
		
	public void grabIntake() {
		motor.set(RobotMap.IntakeIngressSpeed);
	}
	
	public void shootIntake() {
		motor.set(RobotMap.IntakeEgressSpeed);
	}
	
	public void stopIntake() {		
		motor.stopMotor();
    }

    @Override
    protected void initDefaultCommand() {
    }
}
