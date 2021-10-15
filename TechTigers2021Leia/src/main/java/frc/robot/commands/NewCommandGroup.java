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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.RobotOdometry;
import edu.wpi.first.wpilibj.util.Units;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NewCommandGroup extends SequentialCommandGroup {
  public static NewRunMotionProfile mp;

  /** Creates a new NewCommandGroup. */
  public NewCommandGroup(RobotOdometry odometry, Drive driveTrain) {
    // new Pose2d(30, 90, Rotation2d.fromDegrees(-35)) <- center start
    
    // changed ending speed from 2.5 m/s to 0 for now
    mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(30), new Rotation2d()), 0,
        List.of(),
        new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(30), Rotation2d.fromDegrees(0)), 0.0, true, false);
       
     /*   
        mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(30), new Rotation2d()), 0,
        List.of(new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(30)), new Translation2d(Units.inchesToMeters(210), Units.inchesToMeters(120))),
        new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(60), Rotation2d.fromDegrees(-30)), 0.0, false, false);
    */

    /*
    mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(0.762, 0.762, new Rotation2d()), 0,
        List.of(new Translation2d(4.572, 0.762), new Translation2d(5.334, 3.048)),
        new Pose2d(8.382, 1.524, Rotation2d.fromDegrees(-0.762)), 0.0, false, false);
    */
    // Add your addCommands(new FooCommand(), new BarCommand());
    // addCommands(new WaitCommand(2), new BallShooterCommand(1),  new InstantCommand(() -> odometry.setPosition(new Pose2d( Units.inchesToMeters(90),  Units.inchesToMeters(30), new Rotation2d()))), mp);
    addCommands(new WaitCommand(2),  new InstantCommand(() -> odometry.setPosition(new Pose2d( Units.inchesToMeters(90),  Units.inchesToMeters(30), new Rotation2d()))), mp, new BallShooterCommand(1));
  }

  public static void main(String[] args) {
    
    NewCommandGroup cmd = new NewCommandGroup(null, null);
    cmd.mp.visualize(80, List.of(new Translation2d( Units.inchesToMeters(180),  Units.inchesToMeters(30)), new Translation2d( Units.inchesToMeters(210),  Units.inchesToMeters(120)), new Translation2d( Units.inchesToMeters(270),  Units.inchesToMeters(90))));
  }
}
