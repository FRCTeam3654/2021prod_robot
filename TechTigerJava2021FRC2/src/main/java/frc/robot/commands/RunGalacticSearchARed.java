// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.RobotOdometry;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunGalacticSearchARed extends SequentialCommandGroup {

  NewRunMotionProfile mp;

  /** Creates a new RunGalacticSearchARed. */
  public RunGalacticSearchARed(RobotOdometry odometry, Drive driveTrain) {
    // new Pose2d(30, 90, Rotation2d.fromDegrees(10)) <- center start
    /*
    mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(30, 120, Rotation2d.fromDegrees(-25)), 0,
        List.of(new Translation2d(90, 90), new Translation2d(150, 60), new Translation2d(180, 150)),
        new Pose2d(330, 150, new Rotation2d()), 100, false, false);
    // Add your addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(30, 120, new Rotation2d()))), mp);
    */
    mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(0.762, 3.04, Rotation2d.fromDegrees(-25)), 0,
    List.of(new Translation2d(2.286, 2.286), new Translation2d(3.81, 1.524), new Translation2d(4.572, 3.81)),
    new Pose2d(8.382, 3.81, new Rotation2d()), 0, false, false);
    // Add your addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(0.762, 3.04, new Rotation2d()))), mp);
  }

  public static void main(String[] args) {

    RunGalacticSearchARed cmd = new RunGalacticSearchARed(null, null);
    cmd.mp.visualize(80, List.of(new Translation2d(90, 90), new Translation2d(150, 60), new Translation2d(180, 150)));
  }
}
