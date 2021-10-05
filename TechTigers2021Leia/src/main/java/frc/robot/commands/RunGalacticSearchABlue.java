// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import java.util.List;

//import frc.robot.commands.BallPickUpCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.RobotOdometry;
import edu.wpi.first.wpilibj.util.Units;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunGalacticSearchABlue extends SequentialCommandGroup {

  public static NewRunMotionProfile mp;
  public static NewRunMotionProfile mp1;
  public static NewRunMotionProfile mp2;
  public static NewRunMotionProfile mp3;
  public static NewRunMotionProfile mp4;

  /** Creates a new RunGalacticSearchABlue. */
  public RunGalacticSearchABlue(RobotOdometry odometry, Drive driveTrain) {
    // new Pose2d(30, 90, Rotation2d.fromDegrees(-35)) <- center start
    
    // changed ending speed from 2.5 m/s to 0 for now
    /*mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d()), 0,
        List.of(new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(35)), new Translation2d(Units.inchesToMeters(220), Units.inchesToMeters(125)), new Translation2d(Units.inchesToMeters(270), Units.inchesToMeters(90))),
        new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(55), Rotation2d.fromDegrees(-40)), 0.0, false, false);
    */
      mp1 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d()), 0,
        List.of(),
        new Pose2d(Units.inchesToMeters(195), Units.inchesToMeters(50), Rotation2d.fromDegrees(45)), 0.3, false, false);

      mp2 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(195), Units.inchesToMeters(50), Rotation2d.fromDegrees(45)), 0.3,
        List.of(new Translation2d(Units.inchesToMeters(190), Units.inchesToMeters(90))),
        new Pose2d(Units.inchesToMeters(215), Units.inchesToMeters(110), Rotation2d.fromDegrees(-15)), 0.1, false, false);

      mp3 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(225), Units.inchesToMeters(120), Rotation2d.fromDegrees(-15)), 0.1,
        List.of(),
        new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(105), Rotation2d.fromDegrees(-45)), 0.3, false, false);

      mp4 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(105), Rotation2d.fromDegrees(-45)), 0.3,
        List.of(),
        new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(50), Rotation2d.fromDegrees(-45)), 1.3, false, false);
        
    
    // Add your addCommands(new FooCommand(), new BarCommand());
    //addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d( Units.inchesToMeters(30),  Units.inchesToMeters(30), new Rotation2d()))), mp);
    /*addCommands(
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                  new InstantCommand(() -> odometry.setPosition(new Pose2d( Units.inchesToMeters(30),  Units.inchesToMeters(90), new Rotation2d()))), 
                  mp
                  )
                  ,               
            new BallPickUpCommand(1))
        );
        */
    
        addCommands(
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                  new InstantCommand(() -> odometry.setPosition(new Pose2d( Units.inchesToMeters(30),  Units.inchesToMeters(90), new Rotation2d()))), 
                  mp1, mp2, mp3, mp4
                  )
                  ,               
            new BallPickUpCommand(1))
        );
  }

  public static void main(String[] args) {
    
    RunGalacticSearchABlue cmd = new RunGalacticSearchABlue(null, null);
    cmd.mp2.visualize(80, List.of(new Translation2d( Units.inchesToMeters(180),  Units.inchesToMeters(30)), new Translation2d( Units.inchesToMeters(210),  Units.inchesToMeters(120)), new Translation2d( Units.inchesToMeters(270),  Units.inchesToMeters(90))));
  }
}
