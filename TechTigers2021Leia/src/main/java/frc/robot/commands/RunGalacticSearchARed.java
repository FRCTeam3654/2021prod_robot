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
public class RunGalacticSearchARed extends SequentialCommandGroup {

  public static NewRunMotionProfile mp;
  public static NewRunMotionProfile mp1;
  public static NewRunMotionProfile mp2;
  public static NewRunMotionProfile mp3;
  public static NewRunMotionProfile mp4;

  /** Creates a new RunGalacticSearchARed. */
  public RunGalacticSearchARed(RobotOdometry odometry, Drive driveTrain) {
    // new Pose2d(30, 90, Rotation2d.fromDegrees(10)) <- center start
    
    // changed: ending speed from 2.5 m to 0 for now,   changed starting angle -25 to 0
    /*mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), Rotation2d.fromDegrees(0)), 0,
        List.of(new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(90)), new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(75)), new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(50)), new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(40)), new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(90)), new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(145))),
        new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(150), new Rotation2d()), 0, false, false);
    */
      mp1 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), Rotation2d.fromDegrees(0)), 0,
        List.of(),
        new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(90), new Rotation2d()), 0.3, false, false);

      mp2 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(90), Rotation2d.fromDegrees(0)), 0,
        List.of( new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(75))),
        new Pose2d(Units.inchesToMeters(170), Units.inchesToMeters(74), Rotation2d.fromDegrees(45)), 0.3, false, false);

      mp3 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(170), Units.inchesToMeters(60), Rotation2d.fromDegrees(45)), 0,
        List.of(),
        new Pose2d(Units.inchesToMeters(195), Units.inchesToMeters(150), new Rotation2d()), 0.3, false, false);

      mp4 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(195), Units.inchesToMeters(150), Rotation2d.fromDegrees(0)), 0,
        List.of(),
        new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(150), new Rotation2d()), 1.5, false, false);
       
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

    RunGalacticSearchARed cmd = new RunGalacticSearchARed(null, null);
    cmd.mp3.visualize(80, List.of(new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(90)), new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(150))));
  }
}
