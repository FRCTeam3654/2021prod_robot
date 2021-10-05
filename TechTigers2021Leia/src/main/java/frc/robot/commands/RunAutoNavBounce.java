// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import java.util.List;
//import frc.robot.commands.NewRunMotionProfile.CirclePath;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.RobotOdometry;
import edu.wpi.first.wpilibj.util.Units;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// The reversed property simply represents whether the robot is traveling backward. If you specify four waypoints, a, b, c, and d, the robot will still travel in the same order through the waypoints when the reversed flag is set to true. This also means that you must account for the direction of the robot when providing the waypoints. For example, if your robot is facing your alliance station wall and travels backwards to some field element, the starting waypoint should have a rotation of 180 degrees.


public class RunAutoNavBounce extends SequentialCommandGroup {

    NewRunMotionProfile mp1;
    NewRunMotionProfile mp2;
    NewRunMotionProfile mp3;
    NewRunMotionProfile mp4;

    /** Creates a new RunAutoNavBounce. */
    public RunAutoNavBounce(RobotOdometry odometry, Drive driveTrain) {
       
    // test 
    /*
    
    mp1 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d()), 0,
        List.of(), new Pose2d(Units.inchesToMeters(60), Units.inchesToMeters(120), Rotation2d.fromDegrees(90)), 0, false, false);
        
    mp2 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(60), Units.inchesToMeters(120),  Rotation2d.fromDegrees(90)), 0,
       List.of(new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(90))), 
       new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(120), Rotation2d.fromDegrees(270)), 0, true, false);

    mp3 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(120), Rotation2d.fromDegrees(270)), 0,
       List.of(), new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(90), Rotation2d.fromDegrees(360)), 0, false, false);
       

    mp4 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(90),  Rotation2d.fromDegrees(360)), 0,
        List.of(), new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(120), Rotation2d.fromDegrees(270)), 0, true, false);
    
    */
    
    // real path
    mp1 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d()), 0,
               List.of(new Translation2d(Units.inchesToMeters(70), Units.inchesToMeters(90))), new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(150), Rotation2d.fromDegrees(90)), 0, false, false);
   
    // note: final -90 , could be Pigeon's 270
    mp2 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(150), Rotation2d.fromDegrees(90)), 0,
                List.of(new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(30)), new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(60))),
                new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(150), Rotation2d.fromDegrees(270)), 0, true, false);
     // could be: 110, 60 ,  changed -90 to 270

    //note: final +90 could be 360+90 = 450 in Pigeon
    mp3 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(150), Rotation2d.fromDegrees(270)), 0,
                List.of(new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(210), Units.inchesToMeters(30)), new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(30)),  new Translation2d(Units.inchesToMeters(270), Units.inchesToMeters(60))),
                new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(150), Rotation2d.fromDegrees(450)), 0, false, false);
    // could be:  185, 60
     
    //  changed: 90 -> 450, 180 -> 540  ( added 360 degree to match pigeon reading)
    mp4 = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(Units.inchesToMeters(280), Units.inchesToMeters(150), Rotation2d.fromDegrees(450)), 0,
                List.of(new Translation2d(Units.inchesToMeters(290), Units.inchesToMeters(100))), new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(85), Rotation2d.fromDegrees(520)), 0, true,
                false);
    

    
     
        
        addCommands(
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new InstantCommand(() -> odometry.setPosition(new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d()))), 
                    mp1, mp2, mp3, mp4 
                )
                , 
            new StartStopTimer())
        );
        

        /*
        addCommands(
                    new InstantCommand(() -> odometry.setPosition(new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d()))), 
                    mp1 , mp2   , mp3    , mp4
        );
        */
    }

    public static void main(String[] args) {
        RunAutoNavBounce cmd = new RunAutoNavBounce(null, null);
        //cmd.mp1.visualize(80, List.of());
       cmd.mp4.visualize(80, List.of());

        /*
        cmd.mp1.visualize(80, 
         List.of(new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(150)), new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(150)), new
         Translation2d(Units.inchesToMeters(270), Units.inchesToMeters(150)),
         new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(120)), new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(120)), new
         Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(120)),
         new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(120)), new Translation2d(Units.inchesToMeters(210), Units.inchesToMeters(120)), new
         Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(120)),
         new Translation2d(Units.inchesToMeters(300), Units.inchesToMeters(120)), new Translation2d(Units.inchesToMeters(330), Units.inchesToMeters(120)), new
         Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60)),
         new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(210),
         Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(60)),
         new Translation2d(Units.inchesToMeters(300), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(330), Units.inchesToMeters(60)), new Translation2d(Units.inchesToMeters(90),
         Units.inchesToMeters(30))
        ));
        */
    }
}
