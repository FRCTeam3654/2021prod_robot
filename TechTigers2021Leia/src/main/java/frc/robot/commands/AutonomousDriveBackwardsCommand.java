/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;

import java.util.concurrent.atomic.AtomicInteger;

import edu.wpi.first.wpilibj.Timer;


public class AutonomousDriveBackwardsCommand extends CommandBase {
  private boolean autonomousFlag = false;//indicated the first time in loop. used to set initial conditions
  private double startTimeAutonomous = 0;
  private AtomicInteger _pathNumber = new AtomicInteger(1);
  public AutonomousDriveBackwardsCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    addRequirements(RobotContainer.drive);
  }

  public AutonomousDriveBackwardsCommand(int pathNumber) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    addRequirements(RobotContainer.drive);
    _pathNumber = new AtomicInteger(pathNumber);
  }


  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (!autonomousFlag){
   
      startTimeAutonomous = Timer.getFPGATimestamp();
      autonomousFlag = true;

      RobotContainer.drive.setMotionProfile(_pathNumber.get());
      //Robot.drive.setMotionMagic(RobotMap.autonomousTargetPos, 8000, 4000); // not use AuxPID
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if (AutonomousCommandGroup.stopAutonomousCommandGroup == true){
      return true;
    }

    if(autonomousFlag == true) {
      if( (startTimeAutonomous + 0.1) > Timer.getFPGATimestamp()) {
          // if autonomous started, let it run at least 100 ms
          return false;
      }
      else if( RobotContainer.drive.isMotionProfileDone() ) {
          autonomousFlag = false;
          return true;
      }
      else if( !RobotContainer.drive.isMotionProfileDone() || RobotContainer.drive.getCurrentMPState() == 2) {
        // enforce timeout in case MP is stucked or running too long
        if(startTimeAutonomous + RobotMap.autonomousTimeOut < Timer.getFPGATimestamp()) {
           autonomousFlag = false;
           return true;
        }
      }
      else {
        double [] yawPitchRollArray = RobotContainer.drive.getYawPitchRoll();
        
        if( yawPitchRollArray != null && RobotContainer.drive.yawPitchRollArrayStarting != null) {
          // if either yaw change too much,  or pitch/roll (not sure which one)
          //double yawDiff = yawPitchRollArray[0] - Robot.drive.yawPitchRollArrayStarting[0];
          double pitchDiff = yawPitchRollArray[1] - RobotContainer.drive.yawPitchRollArrayStarting[1]; 
          double rollDiff = yawPitchRollArray[2] - RobotContainer.drive.yawPitchRollArrayStarting[2];

          if(  Math.abs(pitchDiff) > 6 || Math.abs(rollDiff) > 6 ) {
               // stop the motion profile to continue if robot is sitting on a ball
               AutonomousCommandGroup.stopAutonomousCommandGroup = true;
               return true;
          }
        }
      }
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

}
