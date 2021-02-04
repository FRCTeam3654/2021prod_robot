/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClimbLockCommand extends Command {
  public ClimbLockCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.ballPickUp);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.oi.climbLockLeftButton.get()){
      Robot.ballPickUp.climbLockLeftSolenoid(false);
    
    }
    if (Robot.oi.climbUnlockLeftButton.get()){
      Robot.ballPickUp.climbLockLeftSolenoid(true);
    }
    if (Robot.oi.climbLockRightButton.get()){
      Robot.ballPickUp.climbLockRightSolenoid(false);
    }
    if (Robot.oi.climbUnlockRightButton.get()){
      Robot.ballPickUp.climbLockRightSolenoid(true);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
