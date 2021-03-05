// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import java.util.List;

import frc.robot.commands.NewRunMotionProfile;
import frc.robot.commands.RunGalacticSearchARed;
import frc.robot.commands.RunGalacticSearchABlue;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.RobotOdometry;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunGalacticSearchB extends CommandBase {
  public boolean galacticRedB;

  NetworkTable mercyLimelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry MercyLimelightx = mercyLimelightTable.getEntry("tx");
  NetworkTableEntry MercyLimelighty = mercyLimelightTable.getEntry("ty");
  NetworkTableEntry MercyLimelightArea = mercyLimelightTable.getEntry("ta");
  private double locationX;
  private double locationY;
  private double locationA;
  private double startTimeLimelight = 0;
  private RobotOdometry odometry;
  private Drive driveTrain;
  
  public RunGalacticSearchB(RobotOdometry odometry, Drive driveTrain) {
    this.odometry = odometry;
    this.driveTrain = driveTrain;
  }


  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    startTimeLimelight = Timer.getFPGATimestamp();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //1 is force off
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1); //setting the pipeline to 1 for finding power cell
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    //read values periodically
    locationX = MercyLimelightx.getDouble(0.0);
    locationY = MercyLimelighty.getDouble(0.0);
    locationA = MercyLimelightArea.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", locationX);
    SmartDashboard.putNumber("LimelightY", locationY);
    SmartDashboard.putNumber("LimelightArea", locationA);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if(startTimeLimelight + 0.25 < Timer.getFPGATimestamp()) {
      if(Math.abs(locationX) < 10){
      galacticRedB = true;
      }else {
      galacticRedB = false;
      }
      if(galacticRedB == true){
        RunGalacticSearchASequential.mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(120), new Rotation2d()), 0,
        List.of(new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(120)), new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(210), Units.inchesToMeters(120))),
        new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(135), Rotation2d.fromDegrees(0)), 0, false, false);
      } else {
        RunGalacticSearchASequential.mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(45), new Rotation2d()), 0,
        List.of(new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(120))),
        new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(30), Rotation2d.fromDegrees(-45)), 0, false, false);
      }
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
