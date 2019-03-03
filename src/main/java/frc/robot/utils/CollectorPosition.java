/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

/**
 * Add your docs here.
 */
public class CollectorPosition {
    public double elevatorPos;
    public double armPos;
    public double wristPos;

    public CollectorPosition(double ep, double ap, double wp){
        elevatorPos = ep;
        armPos = ap;
        wristPos = wp;
    }
}
