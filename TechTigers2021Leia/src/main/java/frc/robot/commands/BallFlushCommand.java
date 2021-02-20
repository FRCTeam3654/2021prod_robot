/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.command.Command;
//import java.util.concurrent.atomic.AtomicInteger;
import frc.robot.Robot;
//import edu.wpi.first.wpilibj.Timer;
//import frc.robot.RobotMap;
//import frc.robot.OI;

public class BallFlushCommand extends Command {
  private boolean isButtonPressed = false;
  private boolean armDown = false;
  public BallFlushCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.ballShooter);
    requires(Robot.ballPickUp);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.oi.ballFlushButton.get())
    {
      //Robot.ballShooter.shoot(true);
      Robot.ballStorage.ballCounter = 0;
      Robot.ballStorage.driveBallStorage4(-0.9);
    }
    else
    {
     // Robot.ballShooter.shoot(false);      
      //Robot.ballShooter.shoot(true);    
      Robot.ballStorage.driveBallStorage4(0);
    }
    if (Robot.oi.ballPickUpButton.get()){
      isButtonPressed = true;
      if (!armDown){
        armDown = true;
      }
      else {
        armDown = false;
      }
    }
      else {
        Robot.ballPickUp.moveArm(false);
      }
    if (!Robot.oi.ballPickUpButton.get()){
      isButtonPressed = false;
    }
  }


  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Robot.oi.ballFlushButton.get()) {
    }
    else
    {
      return true;
    }
      
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
    armDown = false;
 }
 
}
