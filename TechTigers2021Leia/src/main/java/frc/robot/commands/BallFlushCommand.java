/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
//import java.util.concurrent.atomic.AtomicInteger;
import frc.robot.Robot;
//import edu.wpi.first.wpilibj.Timer;
//import frc.robot.RobotMap;
//import frc.robot.OI;
import frc.robot.RobotContainer;

public class BallFlushCommand extends CommandBase {
  private boolean isButtonPressed = false;
  private boolean armDown = false;
  public BallFlushCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    addRequirements(RobotContainer.ballShooter);
    addRequirements(RobotContainer.ballPickUp);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (RobotContainer.oi.ballFlushButton.get())
    {
      //Robot.ballShooter.shoot(true);
      RobotContainer.ballStorage.ballCounter = 0;
      RobotContainer.ballStorage.driveBallStorage1(0.9); //was -0.9
      RobotContainer.ballStorage.driveBallStorage2(-0.9);
    }
    else
    {
     // Robot.ballShooter.shoot(false);      
      //Robot.ballShooter.shoot(true);    
      RobotContainer.ballStorage.driveBallStorage1(0);
      RobotContainer.ballStorage.driveBallStorage2(-0.9);
    }
    if (RobotContainer.oi.ballPickUpButton.get()){
      isButtonPressed = true;
      if (!armDown){
        armDown = true;
      }
      else {
        armDown = false;
      }
    }
      else {
        RobotContainer.ballPickUp.moveArm(false);
      }
    if (!RobotContainer.oi.ballPickUpButton.get()){
      isButtonPressed = false;
    }
  }


  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if(RobotContainer.oi.ballFlushButton.get()) {
    }
    else
    {
      return true;
    }
      
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

 
}
