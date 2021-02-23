/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import java.util.concurrent.atomic.AtomicInteger;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;

public class BallPickUpCommand extends CommandBase {
  private AtomicInteger _mode = new AtomicInteger(0); //mode = 0 means regular teleop; mode = 1 means autonomous mode
  private double startTimeAutonomous = 0;
  private boolean ballPickUpAutonomousFlag = false;
  private double _ballPickUpNewTimeout = RobotMap.autonomousBallPickUpTimeOut;

  private boolean isButtonPressed = false;
  private boolean armDown = false;
  public BallPickUpCommand() {
    addRequirements(RobotContainer.ballPickUp);
    _mode.set(0);
  }

  public BallPickUpCommand(int mode) {
    addRequirements(RobotContainer.ballPickUp);
    _mode.set(mode);
  }

  public BallPickUpCommand(int mode, double ballPickUpNewTimeout) {
    addRequirements(RobotContainer.ballPickUp);
    _mode.set(mode);
    _ballPickUpNewTimeout = ballPickUpNewTimeout;
  }

  @Override
  public void initialize() {
  }

   @Override
  public void execute() {
    //Robot.ballPickUp.moveArm(true);
    //SmartDashboard.putNumber("BallPickUpFlag", 1);

    
        if ((RobotContainer.oi.ballPickUpButton.get() && !isButtonPressed) || _mode.get() == 1){
          isButtonPressed = true;
          if (!armDown){
            armDown = true;
          }
          else {
            armDown = false;
          }
        }
          if ((armDown && _mode.get() != 2)  || _mode.get() == 1){
            RobotContainer.ballPickUp.moveArm(true);
            if (!ballPickUpAutonomousFlag){
              startTimeAutonomous = Timer.getFPGATimestamp();
              ballPickUpAutonomousFlag = true;
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
    if(ballPickUpAutonomousFlag == true && _mode.get() == 1) {
      if( (startTimeAutonomous + 0.1) > Timer.getFPGATimestamp()) {
          // if autonomous started, let it run at least 100 ms
          return false;
      }
      // enforce timeout in case MP is stucked or running too long
      else if(startTimeAutonomous + _ballPickUpNewTimeout < Timer.getFPGATimestamp()) {
          ballPickUpAutonomousFlag = false;
          RobotContainer.ballPickUp.moveArm(false);
          _mode.set(0);  
           return true;
      }
      
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }

  
}
