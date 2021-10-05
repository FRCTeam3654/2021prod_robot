// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class TurretTurningCommand extends CommandBase {
  public TurretTurningCommand() {
    // Use requires() here to declare subsystem dependencies
    addRequirements(RobotContainer.turret);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    RobotContainer.turret.zeroSensor();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    RobotContainer.turret.turretTurning(RobotMap.turretTickAmount);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if (RobotContainer.turret.turretTickCount() >= RobotMap.turretTickAmount){
      RobotContainer.turret.zeroSensor();
      return true;
    }
    return false;
  }
    
  

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    RobotContainer.turret.manualTurret(0.0);
    RobotContainer.turret.zeroSensor();
  }
 
}
