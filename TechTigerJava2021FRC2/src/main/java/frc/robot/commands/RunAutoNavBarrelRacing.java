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
import frc.robot.Constants;
//import frc.robot.Constants.RobotType;
import frc.robot.subsystems.RobotOdometry;



public class RunAutoNavBarrelRacing extends SequentialCommandGroup {
  /** Creates a new RunAutoNavBarrelRacing. */
  
  
  NewRunMotionProfile mp;

  /** Creates a new RunAutoNavBarrelRacing. */
  /*
  public RunAutoNavBarrelRacing(RobotOdometry odometry, DriveSubsystem driveTrain) {
    mp = new NewRunMotionProfile(driveTrain, odometry, 0.0,
        List.of(new Pose2d(0.763, 2.286, new Rotation2d()),
            new CirclePath(new Translation2d(3.81, 1.524), 0.762, new Rotation2d(), Rotation2d.fromDegrees(-180), true),
            new CirclePath(new Translation2d(6.096, 3.048), 0.762, new Rotation2d(), Rotation2d.fromDegrees(180), false),
            new CirclePath(new Translation2d(7.62, 1.524), 0.762, Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(90),
                false),
            new Pose2d(3.81, 2.286, Rotation2d.fromDegrees(180)), new Pose2d(1.067, 2.286, Rotation2d.fromDegrees(180))),
        1.0, false, false);
    // Add your addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(0.763, 2.286, new Rotation2d()))), mp,
        new InstantCommand(() -> driveTrain.stop()));
  }
  */
  
 // public RunAutoNavBarrelRacing(RobotOdometry odometry, DriveSubsystem driveTrain) {
  public RunAutoNavBarrelRacing(RobotOdometry odometry, Drive driveTrain) {


    mp = new NewRunMotionProfile(driveTrain, odometry, 0.0,
        List.of(new Pose2d(0.763, 2.286, new Rotation2d()),
         new CirclePath(new Translation2d(3.81, 1.524), 0.762, new Rotation2d(), Rotation2d.fromDegrees(-180), true),
         new CirclePath(new Translation2d(6.096, 3.048), 0.762, new Rotation2d(), Rotation2d.fromDegrees(180), false),
         new CirclePath(new Translation2d(7.62, 1.524), 0.762, Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(90), false),
         new Pose2d(3.81, 2.286, Rotation2d.fromDegrees(180)), new Pose2d(1.067, 2.286, Rotation2d.fromDegrees(180))
        //List.of(new Pose2d(0.0, 0.0, new Rotation2d()),
        // new CirclePath(new Translation2d(1.0, 0.762), 0.762, new Rotation2d(), Rotation2d.fromDegrees(180), false)  // working
        //new Pose2d(1, 0, Rotation2d.fromDegrees(0))
        // new Pose2d(1, 0.762, Rotation2d.fromDegrees(90)) // working
        //new Pose2d(0.238, 1.524, Rotation2d.fromDegrees(180)) // working
        //new Pose2d(0.762, 1.524, Rotation2d.fromDegrees(-90))
        ),  
        0.0, false, false);
        
        
    addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(0.763, 2.286, new Rotation2d()))), mp,
        new InstantCommand(() -> driveTrain.stop()));

  }


  public static void main(String[] args) {

    RunAutoNavBarrelRacing cmd = new RunAutoNavBarrelRacing(null, null);
    cmd.mp.visualize(80, List.of());
    // List.of(new Translation2d(30, 120), new Translation2d(60, 120), new
    // Translation2d(30, 60),
    // new Translation2d(60, 60), new Translation2d(150, 60), new Translation2d(240,
    // 120),
    // new Translation2d(300, 60))
  }

}
