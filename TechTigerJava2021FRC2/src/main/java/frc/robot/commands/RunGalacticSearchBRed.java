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
import edu.wpi.first.wpilibj.util.Units;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunGalacticSearchBRed extends SequentialCommandGroup {

  NewRunMotionProfile mp;

  /** Creates a new RunGalacticSearchBRed. */
  public RunGalacticSearchBRed(RobotOdometry odometry, Drive driveTrain) {
    // new Pose2d(30, 90, Rotation2d.fromDegrees(35)) <- center start
  
    // changed: ending speed to 0 for now,  need change intital Y: 120 or 135?  changed to 120
    mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(120), new Rotation2d()), 0,
        List.of(new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(120)), new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(210), Units.inchesToMeters(120))),
        new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(135), Rotation2d.fromDegrees(0)), 0, false, false);
    // Add your addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(120), new Rotation2d()))), mp);
    

    /*
    mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(0.762, 3.429, new Rotation2d()), 0,
        List.of(new Translation2d(2.286, 3.048), new Translation2d(3.81, 1.524), new Translation2d(5.334, 3.048)),
        new Pose2d(8.382, 3.429, Rotation2d.fromDegrees(0)), 0, false, false);
    // Add your addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(0.762, 3.429, new Rotation2d()))), mp);
    */
  }

  public static void main(String[] args) {
    RunGalacticSearchBRed cmd = new RunGalacticSearchBRed(null, null);
    cmd.mp.visualize(80, List.of(new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(120)), new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(210), Units.inchesToMeters(120))));
  }
}
