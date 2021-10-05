/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ClimbLockCommand extends CommandBase {
  public ClimbLockCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    addRequirements(RobotContainer.ballPickUp);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (RobotContainer.oi.climbLockLeftButton.get()){
      RobotContainer.ballPickUp.climbLockLeftSolenoid(false);
    
    }
    if (RobotContainer.oi.climbUnlockLeftButton.get()){
      RobotContainer.ballPickUp.climbLockLeftSolenoid(true);
    }
    if (RobotContainer.oi.climbLockRightButton.get()){
      RobotContainer.ballPickUp.climbLockRightSolenoid(false);
    }
    if (RobotContainer.oi.climbUnlockRightButton.get()){
      RobotContainer.ballPickUp.climbLockRightSolenoid(true);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

}
