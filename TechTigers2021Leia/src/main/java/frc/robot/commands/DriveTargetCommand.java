/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import frc.robot.Robot;
import frc.robot.RobotMap;
//import jdk.internal.util.xml.impl.Input;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;

public class DriveTargetCommand extends CommandBase {
  NetworkTable mercyLimelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry MercyLimelightx = mercyLimelightTable.getEntry("tx");
  NetworkTableEntry MercyLimelighty = mercyLimelightTable.getEntry("ty");
  NetworkTableEntry MercyLimelightArea = mercyLimelightTable.getEntry("ta");
  private double readJoeyX = 2;
  private double readJoeyY = 0;
  private double startTimeLimelight = 0;

  public DriveTargetCommand() {
    addRequirements(RobotContainer.drive);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    startTimeLimelight = Timer.getFPGATimestamp();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); //3 is force on
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0); //setting the pipeline to 0 for targetting the goal
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double turn90X;
    double turn90Y;

    //read values periodically
    readJoeyX = MercyLimelightx.getDouble(0.0);
    readJoeyY = MercyLimelighty.getDouble(0.0);
    double area = MercyLimelightArea.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", readJoeyX);
    SmartDashboard.putNumber("LimelightY", readJoeyY);
    SmartDashboard.putNumber("LimelightArea", area);

    turn90X = readJoeyX * RobotMap.LimelightJoeyX; // A little drive forward
    turn90X = Math.min(0.1, turn90X);
    turn90X = Math.max(-0.1, turn90X);
    turn90Y = (readJoeyY - 20)/150;   //ratio to  determinte how much to turn based on limelight Input
    turn90Y = Math.min(0.3, turn90Y);
    turn90Y = Math.max(-0.3, turn90Y);
    SmartDashboard.putNumber("LimelightSpeed", turn90X);
    RobotContainer.drive.setArcade(turn90X, turn90Y);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if(startTimeLimelight + 0.25 < Timer.getFPGATimestamp()) {
      if(Math.abs(readJoeyX) < 2){
        return true;
      }
    }else {
      return false;
    }
    if(startTimeLimelight + RobotMap.limeLightTimeout < Timer.getFPGATimestamp()) {   // to much time to turn to target
      return true;
    }
    if(RobotContainer.oi.turboButton.get()){ // pressing Turbo breaks this mode
      return true;
    } 
  return false;
}

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //1 is force off LED - required by FRC
  }

}
