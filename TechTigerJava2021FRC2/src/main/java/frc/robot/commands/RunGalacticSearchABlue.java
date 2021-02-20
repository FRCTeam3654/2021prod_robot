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
public class RunGalacticSearchABlue extends SequentialCommandGroup {

  NewRunMotionProfile mp;

  /** Creates a new RunGalacticSearchABlue. */
  public RunGalacticSearchABlue(RobotOdometry odometry, Drive driveTrain) {
    // new Pose2d(30, 90, Rotation2d.fromDegrees(-35)) <- center start
    /*
    mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(30, 30, new Rotation2d()), 0,
        List.of(new Translation2d(180, 30), new Translation2d(210, 120)),
        new Pose2d(330, 60, Rotation2d.fromDegrees(-30)), 100, false, false);
        */
    mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(0.762, 0.762, new Rotation2d()), 0,
        List.of(new Translation2d(4.572, 0.762), new Translation2d(5.334, 3.048)),
        new Pose2d(8.382, 1.524, Rotation2d.fromDegrees(-0.762)), 0.0, false, false);

    // Add your addCommands(new FooCommand(), new BarCommand());
   // addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(30, 35, new Rotation2d()))), mp);
    addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(0.762, 0.762, new Rotation2d()))), mp);
  }

  public static void main(String[] args) {
    
    RunGalacticSearchABlue cmd = new RunGalacticSearchABlue(null, null);
    //cmd.mp.visualize(2.0, List.of(new Translation2d(180, 30), new Translation2d(210, 120), new Translation2d(270, 90)));
  }
}
