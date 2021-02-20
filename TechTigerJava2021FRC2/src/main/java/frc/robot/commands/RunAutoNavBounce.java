// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import java.util.List;
import frc.robot.commands.NewRunMotionProfile.CirclePath;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.RobotOdometry;
import edu.wpi.first.wpilibj.util.Units;

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
       

    
    mp1 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d()), 0,
               List.of(new Translation2d(Units.inchesToMeters(70), Units.inchesToMeters(90))), new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(150), Rotation2d.fromDegrees(90)), 0, false, false);
   /*
    mp2 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(150), Rotation2d.fromDegrees(90)), 0,
                List.of(new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(30)), new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(60))),
                new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(150), Rotation2d.fromDegrees(-90)), 0, true, false);
     */

    /*
                mp3 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(150), new Rotation2d(-90)), 0,
                List.of(new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(210), Units.inchesToMeters(30)), new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(30)),  new Translation2d(Units.inchesToMeters(270), Units.inchesToMeters(60))),
                new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(150), Rotation2d.fromDegrees(90)), 0, false, false);
    
     

        mp4 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(280), Units.inchesToMeters(150), new Rotation2d(90)), 0,
                List.of(new Translation2d(Units.inchesToMeters(290), Units.inchesToMeters(100))), new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(100), Rotation2d.fromDegrees(180)), 0, true,
                false);
     */

        /// mp4 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(280), Units.inchesToMeters(150), new Rotation2d(0)), 0,
         //       List.of(new Translation2d(Units.inchesToMeters(290), Units.inchesToMeters(100))), new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(100), Rotation2d.fromDegrees(90)), 0, true,
         //       false);
     
       

        
        // Add your addCommands(new FooCommand(), new BarCommand());
        //addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(30, 90, new Rotation2d()))), mp1, mp2, mp3, mp4);
        //addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d()))), mp1, mp2);

        //addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(150), new Rotation2d(-90)))),  mp3);

        addCommands(
            //new MotionMagicDriveCommand(1.2, false),
            new InstantCommand(() -> odometry.setPosition(new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d()))), 
            mp1 
           // new InstantCommand(() -> odometry.setPosition(new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(150), Rotation2d.fromDegrees(90)))), 
            //mp2

            //new InstantCommand(() -> odometry.setPosition(new Pose2d(Units.inchesToMeters(280), Units.inchesToMeters(150), new Rotation2d(0)))), 
            //mp4

            //new MotionMagicDriveCommand(1.2, true)
            //new MotionMagicDriveCommand(2.2, true)
        );



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
