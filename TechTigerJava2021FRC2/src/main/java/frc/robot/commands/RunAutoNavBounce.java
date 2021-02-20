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
public class RunAutoNavBounce extends SequentialCommandGroup {

    NewRunMotionProfile mp1;
    NewRunMotionProfile mp2;
    NewRunMotionProfile mp3;
    NewRunMotionProfile mp4;

    /** Creates a new RunAutoNavBounce. */
    public RunAutoNavBounce(RobotOdometry odometry, Drive driveTrain) {

        mp1 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(0.762, 2.286, new Rotation2d()), 0,
                List.of(new Translation2d(1.778, 2.286)), new Pose2d(2.286, 3.81, Rotation2d.fromDegrees(90)), 0, false, false);
       

        /*
        mp1 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(30, 90, new Rotation2d()), 0,
                List.of(new Translation2d(70, 90)), new Pose2d(90, 150, Rotation2d.fromDegrees(90)), 0, false, false);
       
        
        mp2 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(90, 150, new Rotation2d(90)), 0,
                List.of(new Translation2d(120, 60), new Translation2d(150, 30), new Translation2d(180, 60)),
                new Pose2d(180, 150, Rotation2d.fromDegrees(-90)), 0, true, false);
        mp3 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(180, 150, new Rotation2d(-90)), 0,
                List.of(new Translation2d(180, 60), new Translation2d(200, 38), new Translation2d(270, 38)),
                new Pose2d(280, 150, Rotation2d.fromDegrees(90)), 0, false, false);
        mp4 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(280, 150, new Rotation2d(90)), 0,
                List.of(new Translation2d(290, 100)), new Pose2d(330, 100, Rotation2d.fromDegrees(180)), 0, true,
                false);
        */
        // Add your addCommands(new FooCommand(), new BarCommand());
        //addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(30, 90, new Rotation2d()))), mp1, mp2, mp3, mp4);
        addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(0.762, 2.286, new Rotation2d()))), mp1);
    }

    public static void main(String[] args) {
        RunAutoNavBounce cmd = new RunAutoNavBounce(null, null);
        //cmd.mp1.visualize(80, List.of());
        // List.of(new Translation2d(90, 150), new Translation2d(180, 150), new
        // Translation2d(270, 150),
        // new Translation2d(30, 120), new Translation2d(60, 120), new
        // Translation2d(120, 120),
        // new Translation2d(150, 120), new Translation2d(210, 120), new
        // Translation2d(240, 120),
        // new Translation2d(300, 120), new Translation2d(330, 120), new
        // Translation2d(30, 60), new Translation2d(60, 60),
        // new Translation2d(90, 60), new Translation2d(150, 60), new Translation2d(210,
        // 60), new Translation2d(240, 60),
        // new Translation2d(300, 60), new Translation2d(330, 60), new Translation2d(90,
        // 30))
    }
}
