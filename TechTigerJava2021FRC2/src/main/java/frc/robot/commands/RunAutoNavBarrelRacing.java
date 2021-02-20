// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
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
        List.of(new Pose2d(30.0, 90.0, new Rotation2d()),
            new CirclePath(new Translation2d(150, 60), 30, new Rotation2d(), Rotation2d.fromDegrees(-180), true),
            new CirclePath(new Translation2d(240, 120), 30, new Rotation2d(), Rotation2d.fromDegrees(180), false),
            new CirclePath(new Translation2d(300, 60), 30, Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(90),
                false),
            new Pose2d(150.0, 90, Rotation2d.fromDegrees(180)), new Pose2d(42.0, 90.0, Rotation2d.fromDegrees(180))),
        130.0, false, false);
    // Add your addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(30, 90, new Rotation2d()))), mp,
        new InstantCommand(() -> driveTrain.stop()));
  }
  */
  
 // public RunAutoNavBarrelRacing(RobotOdometry odometry, DriveSubsystem driveTrain) {
  public RunAutoNavBarrelRacing(RobotOdometry odometry, Drive driveTrain) {
    /*
    mp = new NewRunMotionProfile(driveTrain, odometry, 0.0,
        List.of(new Pose2d(0.0, 0.0, new Rotation2d()),
            new Pose2d(2, 0, Rotation2d.fromDegrees(0))),
        0.0, false, false);
    */

    mp = new NewRunMotionProfile(driveTrain, odometry, 0.0,
        List.of(new Pose2d(0.0, 0.0, new Rotation2d()),
        // new CirclePath(new Translation2d(1.0, 0.762), 0.762, new Rotation2d(), Rotation2d.fromDegrees(180), false)  // working
        new Pose2d(1, 0, Rotation2d.fromDegrees(0))
        // new Pose2d(1, 0.762, Rotation2d.fromDegrees(90)) // working
        //new Pose2d(0.238, 1.524, Rotation2d.fromDegrees(180)) // working
        //new Pose2d(0.762, 1.524, Rotation2d.fromDegrees(-90))
        ),  
        0.0, false, false);
        
        
    // Add your addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(0.0, 0.0, new Rotation2d()))), mp,
        new InstantCommand(() -> driveTrain.stop()));

    System.err.println("to driver now");
   // addCommands(new InstantCommand(() -> driveTrain.arcadeDrive(0.3, 0)));
  }



}
