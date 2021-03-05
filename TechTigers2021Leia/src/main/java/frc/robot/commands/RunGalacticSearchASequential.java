// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.RobotOdometry;
import edu.wpi.first.wpilibj.util.Units;

public class RunGalacticSearchASequential extends SequentialCommandGroup {
  /** Creates a new RunGalacticSearchASequential. */

  public static NewRunMotionProfile mp;

  public RunGalacticSearchASequential(RobotOdometry odometry, Drive driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    RunGalacticSearchA galacticA = new RunGalacticSearchA(odometry, driveTrain);
    addCommands(galacticA);
    addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d( Units.inchesToMeters(30),  Units.inchesToMeters(30), new Rotation2d()))));
    addCommands(mp);
  }
}