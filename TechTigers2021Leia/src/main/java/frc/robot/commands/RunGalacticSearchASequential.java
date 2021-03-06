// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.RobotOdometry;
import edu.wpi.first.wpilibj.util.Units;
import java.util.List;

public class RunGalacticSearchASequential extends SequentialCommandGroup {
  /** Creates a new RunGalacticSearchASequential. */

  public static NewRunMotionProfile mp;

  public RunGalacticSearchASequential(RobotOdometry odometry, Drive driveTrain){
    mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(120), Rotation2d.fromDegrees(0)), 0,
    List.of(new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(90)), new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(150))),
    new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(150), new Rotation2d()), 0, false, false);

    addCommands(
      new ParallelDeadlineGroup(
          new SequentialCommandGroup(
            new RunGalacticSearchA(odometry, driveTrain)
            //new InstantCommand(() -> odometry.setPosition(new Pose2d( Units.inchesToMeters(30),  Units.inchesToMeters(30), new Rotation2d())))
          )
          , 
      new BallPickUpCommand(1))
    );

  }
}