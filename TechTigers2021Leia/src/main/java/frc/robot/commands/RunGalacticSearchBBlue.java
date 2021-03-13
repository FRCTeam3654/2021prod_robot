// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import java.util.List;

import frc.robot.commands.BallPickUpCommand;
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
public class RunGalacticSearchBBlue extends SequentialCommandGroup {

  NewRunMotionProfile mp;

  /** Creates a new RunGalacticSearchBBlue. */
  public RunGalacticSearchBBlue(RobotOdometry odometry, Drive driveTrain) {
    // new Pose2d(30, 90, Rotation2d.fromDegrees(-20)) <- center start
    
    // changed: ending speend to 0,  and initial postion from 65 to 45  (addCommands)
    mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(45), new Rotation2d()), 0,
        List.of(new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(120))),
        new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(30), Rotation2d.fromDegrees(-45)), 0, false, false);
    addCommands(
       new ParallelDeadlineGroup(
           new SequentialCommandGroup(
             new InstantCommand(() -> odometry.setPosition(new Pose2d( Units.inchesToMeters(30),  Units.inchesToMeters(45), new Rotation2d()))), 
                mp
                )
                ,               
      new BallPickUpCommand(1))
      );
  }

  public static void main(String[] args) {
    RunGalacticSearchBBlue cmd = new RunGalacticSearchBBlue(null, null);
    cmd.mp.visualize(80, List.of(new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(120)), new Translation2d(Units.inchesToMeters(300), Units.inchesToMeters(60))));
  }
}
