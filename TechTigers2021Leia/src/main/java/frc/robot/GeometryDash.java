/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//geometry dash is the best game
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Robot;
//import frc.robot.RobotMap;
/**
 * Add your docs here.
 */
public class GeometryDash {
    public void dashPush(){
        SmartDashboard.putNumber("Autonomous Velocity", RobotMap.autonomousVelocity);
        SmartDashboard.putNumber("Maximum Talon Velocity", RobotMap.maximumVelocityFalcon);
        SmartDashboard.putNumber("Non-Turbo Multipliers Forward", RobotMap.nonTurboMultiplierForward);
        SmartDashboard.putNumber("Non-Turbo Multipliers Turn", RobotMap.nonTurboMultiplierTurn);
        
    }
}
