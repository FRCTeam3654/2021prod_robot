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
public class RunGalacticSearchBRed extends SequentialCommandGroup {

  NewRunMotionProfile mp;
  NewRunMotionProfile mp1;
  NewRunMotionProfile mp2;
  NewRunMotionProfile mp3;
  NewRunMotionProfile mp4;

  /** Creates a new RunGalacticSearchBRed. */
  public RunGalacticSearchBRed(RobotOdometry odometry, Drive driveTrain) {
    // new Pose2d(30, 90, Rotation2d.fromDegrees(35)) <- center start
  
    // changed: ending speed to 0 for now,  need change intital Y: 120 or 135?  changed to 120
    /*mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(120), new Rotation2d()), 0,
        List.of(new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(120)), new Translation2d(Units.inchesToMeters(110), Units.inchesToMeters(105)), new Translation2d(Units.inchesToMeters(160), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(170), Units.inchesToMeters(55)), new Translation2d(Units.inchesToMeters(187), Units.inchesToMeters(90)), new Translation2d(Units.inchesToMeters(220), Units.inchesToMeters(120)), new Translation2d(Units.inchesToMeters(245), Units.inchesToMeters(130))),
        new Pose2d(Units.inchesToMeters(340), Units.inchesToMeters(135), Rotation2d.fromDegrees(0)), 0, false, false);
    */
    mp1 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(120), new Rotation2d()), 0,
        List.of(),
        new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(120), Rotation2d.fromDegrees(0)), 0, false, false);

     mp2 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(120), Rotation2d.fromDegrees(0)), 0,
        List.of( new Translation2d(Units.inchesToMeters(110), Units.inchesToMeters(105))),
        new Pose2d(Units.inchesToMeters(160), Units.inchesToMeters(70), Rotation2d.fromDegrees(0)), 0, false, false);

    mp3 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(160), Units.inchesToMeters(70), Rotation2d.fromDegrees(0)), 0,
        List.of( new Translation2d(Units.inchesToMeters(187), Units.inchesToMeters(90))), 
        new Pose2d(Units.inchesToMeters(210), Units.inchesToMeters(120), Rotation2d.fromDegrees(0)), 0, false, false);

    mp4 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(210), Units.inchesToMeters(120), Rotation2d.fromDegrees(0)), 0,
        List.of(),
        new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(110), Rotation2d.fromDegrees(0)), 0, false, false);
    
    /*    addCommands(
       new ParallelDeadlineGroup(
           new SequentialCommandGroup(
             new InstantCommand(() -> odometry.setPosition(new Pose2d( Units.inchesToMeters(30),  Units.inchesToMeters(120), new Rotation2d()))), 
                mp
                )
                ,               
      new BallPickUpCommand(1))
      );
      */
      addCommands(
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
              new InstantCommand(() -> odometry.setPosition(new Pose2d( Units.inchesToMeters(30),  Units.inchesToMeters(120), new Rotation2d()))), 
                 mp1, mp2, mp3, mp4
                 )
                 ,               
       new BallPickUpCommand(1))
       );
  }

  public static void main(String[] args) {
    RunGalacticSearchBRed cmd = new RunGalacticSearchBRed(null, null);
    cmd.mp2.visualize(80, List.of(new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(120)), new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(210), Units.inchesToMeters(120))));
  }
}
