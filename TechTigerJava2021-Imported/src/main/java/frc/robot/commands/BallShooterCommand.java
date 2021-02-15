/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import java.util.concurrent.atomic.AtomicInteger;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;

public class BallShooterCommand extends Command {
  private AtomicInteger _mode = new AtomicInteger(0); //mode = 0 means regular teleop; mode = 1 means autonomous mode
  private double startTimeAutonomous = 0;
  private boolean ballShooterAutonomousFlag = false;

  public BallShooterCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.ballShooter);
    _mode.set(0);
  }

  public BallShooterCommand(int mode) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.ballShooter);
    requires(Robot.ballStorage);
    _mode.set(mode);
  }
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.oi.ballShooterButton.get() || _mode.get() == 1)
    {
      Robot.ballShooter.shoot(true);
      Robot.ballStorage.ballCounter = 0;
      Robot.ballStorage.driveBallStorage4(1.0);


      if (!ballShooterAutonomousFlag){
   
        startTimeAutonomous = Timer.getFPGATimestamp();
        ballShooterAutonomousFlag = true;
        _mode.set(1);
  
        
      }
    }
    else
    {
      Robot.ballShooter.shoot(false);      
      //Robot.ballShooter.shoot(true);    
      Robot.ballStorage.driveBallStorage4(0);

    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(ballShooterAutonomousFlag == true && ( _mode.get() == 1)) {
      if( (startTimeAutonomous + 0.1) > Timer.getFPGATimestamp()) {
          // if autonomous started, let it run at least 100 ms
          return false;
      }
      // enforce timeout in case MP is stucked or running too long
      else if(startTimeAutonomous + RobotMap.autonomousBallShooterTimeOut < Timer.getFPGATimestamp()) {
          ballShooterAutonomousFlag = false;
          //Robot.ballShooter.shoot(false);  
          Robot.ballStorage.driveBallStorage4(0); 
          _mode.set(0);
           return true;
      }
      
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
  }
}
