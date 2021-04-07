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
import frc.robot.commands.NewRunMotionProfile.CirclePath;
import frc.robot.subsystems.RobotOdometry;
import edu.wpi.first.wpilibj.util.Units;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunAutoNavSlalom extends SequentialCommandGroup {

  NewRunMotionProfile mp;

  /** Creates a new RunAutoNavSlalom. */
  public RunAutoNavSlalom(RobotOdometry odometry, Drive driveTrain) {
    /*
    mp = new NewRunMotionProfile(driveTrain, odometry, 0.0,
        List.of(new Pose2d(30.0, 30.0, new Rotation2d()), new Pose2d(90.0, 60.0, Rotation2d.fromDegrees(45.0)),
            new Pose2d(180.0, 90.0, new Rotation2d()),
            new CirclePath(new Translation2d(300, 60), 30, Rotation2d.fromDegrees(-160), Rotation2d.fromDegrees(160),
                false),
            new Pose2d(180.0, 30.0, Rotation2d.fromDegrees(-180.0)),
            new Pose2d(90.0, 60.0, Rotation2d.fromDegrees(90.0 + 45.0)),
            new Pose2d(60.0, 90.0, Rotation2d.fromDegrees(90.0 + 45.0))),
        130.0, false, false);
        */
 
    /*
    mp = new NewRunMotionProfile(driveTrain, odometry, 0.0,
        List.of(new Pose2d(0.762, 0.762, new Rotation2d()), new Pose2d(2.286, 1.524, Rotation2d.fromDegrees(45.0)),
            new Pose2d(4.572, 2.286, new Rotation2d()),
            new CirclePath(new Translation2d(7.62, 1.524), 0.762, Rotation2d.fromDegrees(-160), Rotation2d.fromDegrees(160),false),
            new Pose2d(4.572, 0.762, Rotation2d.fromDegrees(-180.0)),
            new Pose2d(2.286, 1.524, Rotation2d.fromDegrees(90.0 + 45.0)),
            new Pose2d(1.524, 2.286, Rotation2d.fromDegrees(90.0 + 45.0))),
        0.0, false, false);
    */

    mp = new NewRunMotionProfile(driveTrain, odometry, 0.0,
        List.of(new Pose2d(Units.inchesToMeters(30.0), Units.inchesToMeters(30.0), new Rotation2d()), new Pose2d(Units.inchesToMeters(90.0), Units.inchesToMeters(60.0), Rotation2d.fromDegrees(45.0)),
            new Pose2d(Units.inchesToMeters(180.0), Units.inchesToMeters(90.0), new Rotation2d()),
            new CirclePath(new Translation2d(Units.inchesToMeters(300), Units.inchesToMeters(60)), Units.inchesToMeters(30), Rotation2d.fromDegrees(-160), Rotation2d.fromDegrees(160),
                false),
            new Pose2d(Units.inchesToMeters(180.0), Units.inchesToMeters(30.0), Rotation2d.fromDegrees(180.0)),
            new Pose2d(Units.inchesToMeters(93.0), Units.inchesToMeters(60.0), Rotation2d.fromDegrees(90.0 + 45.0)),
            new Pose2d(Units.inchesToMeters(65.0), Units.inchesToMeters(95.0), Rotation2d.fromDegrees(90.0 + 60.0))),
        0, false, false);
    


    // Add your addCommands(new FooCommand(), new BarCommand());
  addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(30), new Rotation2d()))), mp, new InstantCommand(() -> driveTrain.stop()));

   

  }

  public static void main(String[] args) {
    RunAutoNavSlalom cmd = new RunAutoNavSlalom(null, null);
    //cmd.mp.visualize(80, List.of());
    cmd.mp.visualize(80, 
     List.of(
       
     new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(120)), new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(120)), 
     new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60)), 
     new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(60)), 
     new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(210), Units.inchesToMeters(60)), 
     new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(300), Units.inchesToMeters(60))
     
    ));
  }
}
