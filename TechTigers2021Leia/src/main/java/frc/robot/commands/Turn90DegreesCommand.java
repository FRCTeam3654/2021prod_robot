/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//  import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Robot;
import frc.robot.RobotMap;
//  import java.math.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;

public class Turn90DegreesCommand extends CommandBase {
  private boolean turn90Flag = false;//indicated the first time in loop. used to set initial conditions
  private double turnEndpointAngle = 0;//vinnies desired finish endpoint angle 
  private double vinniesError = 2;//greater than 1 so it doesn't get triggered until the error is correct
  private double startTime90degree = 0;
  private double desiredTurnAngle = 0; 

  public Turn90DegreesCommand() {
    // Use requires() here to declare subsystem dependencies
    addRequirements(RobotContainer.drive);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double turn90X;
    double turn90Y;
    double [] yawPitchRollArray;
    yawPitchRollArray = new double [3];
    RobotContainer.drive.pigeonVinnie.getYawPitchRoll(yawPitchRollArray);
      if (!turn90Flag){
        if (RobotContainer.oi.turnLeft180Button.get()){
          desiredTurnAngle = 180;
        }
        if (RobotContainer.oi.turnRight180Button.get()){
          desiredTurnAngle = - 180;
        }
        if (RobotContainer.oi.turnLeft90Button.get()){
          desiredTurnAngle = 90;
        }
        if (RobotContainer.oi.turnRight90Button.get()){
          desiredTurnAngle = - 90;
        }
        turnEndpointAngle = yawPitchRollArray[0] + desiredTurnAngle;
        startTime90degree = Timer.getFPGATimestamp();
        turn90Flag = true;
      }
      vinniesError = turnEndpointAngle - yawPitchRollArray[0];
      if (Math.abs(vinniesError) > 90){   // for large angles we tune it down a bit
        if (vinniesError > 0){
          //vinniesError = 60;
          turn90X = vinniesError * RobotMap.turnDegreeProportion * 0.75;
        }
        else {
          //vinniesError = -60;
          turn90X = vinniesError * RobotMap.turnDegreeProportion * 0.75;
        }
      }
      else {
        turn90X = vinniesError * RobotMap.turnDegreeProportion;
      }
       turn90X = Math.min(1, turn90X);
       turn90X = Math.max(-1, turn90X);
       turn90Y = 0;
       RobotContainer.drive.setArcade(turn90X, turn90Y);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if(Math.abs(vinniesError) < 3){  // Nearly at desired angle
      turn90Flag = false;
      return true;
    }
    if(startTime90degree + RobotMap.turn90DegreeTimeout < Timer.getFPGATimestamp()) {  // to much time to find desired angle
      turn90Flag = false;
      return true;
    }
    if(RobotContainer.oi.turboButton.get()){   // Pressing Turbo buttion stops turning
      turn90Flag = false;
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

  
}
