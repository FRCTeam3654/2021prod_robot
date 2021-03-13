/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.concurrent.atomic.AtomicInteger;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;

public class BallShooterCommand extends CommandBase {
  private AtomicInteger _mode = new AtomicInteger(0); //mode = 0 means regular teleop; mode = 1 means autonomous mode
  private double startTimeAutonomous = 0;
  private boolean ballShooterAutonomousFlag = false;

  public BallShooterCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    addRequirements(RobotContainer.ballStorage);
    addRequirements(RobotContainer.ballShooter);
    _mode.set(0);
  }

  public BallShooterCommand(int mode) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    addRequirements(RobotContainer.ballShooter);
    addRequirements(RobotContainer.ballStorage);
    _mode.set(mode);
  }
  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    startTimeAutonomous = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (RobotContainer.oi.ballShooterButton.get() || _mode.get() == 1)
    {
      RobotContainer.ballShooter.shoot(true);
      RobotContainer.ballStorage.ballCounter = 0;
      
      if (RobotContainer.ballShooter.targetSpeed()){
        RobotContainer.ballStorage.driveBallStorage1(-1.0);//-1  to move belt forward //1.0
        RobotContainer.ballStorage.driveBallStorage2(-0.5);//-0.5
      }
      


      if (!ballShooterAutonomousFlag){
        ballShooterAutonomousFlag = true;
        _mode.set(1);
      }
    }
    else
    {
      RobotContainer.ballShooter.shoot(false); 
      RobotContainer.ballStorage.driveBallStorage1(0); 
      RobotContainer.ballStorage.driveBallStorage2(0);

    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if(ballShooterAutonomousFlag == true && ( _mode.get() == 1)) {
      if( (startTimeAutonomous + 0.1) > Timer.getFPGATimestamp()) {
          // if autonomous started, let it run at least 100 ms
          return false;
      }
      // enforce timeout in case MP is stucked or running too long
      else if(startTimeAutonomous + RobotMap.autonomousBallShooterTimeOut < Timer.getFPGATimestamp()) {
          ballShooterAutonomousFlag = false;
          //Robot.ballShooter.shoot(false);  
          RobotContainer.ballStorage.driveBallStorage1(0);
          RobotContainer.ballStorage.driveBallStorage2(0); 
          _mode.set(0);
           return true;
      }
      
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    ballShooterAutonomousFlag = false;
    RobotContainer.ballShooter.shoot(false);  
    RobotContainer.ballStorage.driveBallStorage1(0);
    RobotContainer.ballStorage.driveBallStorage2(0); 
  }

}
