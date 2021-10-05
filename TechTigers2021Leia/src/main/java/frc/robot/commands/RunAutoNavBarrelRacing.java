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
//import frc.robot.Constants;
//import frc.robot.Constants.RobotType;
import frc.robot.subsystems.RobotOdometry;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;


public class RunAutoNavBarrelRacing extends SequentialCommandGroup {
  /** Creates a new RunAutoNavBarrelRacing. */
  
  
  NewRunMotionProfile mp;
  
  public RunAutoNavBarrelRacing(RobotOdometry odometry, Drive driveTrain) {

  /*
    mp = new NewRunMotionProfile(driveTrain, odometry, 0.0,
        List.of(new Pose2d(0.763, 2.286, new Rotation2d()),
         new CirclePath(new Translation2d(3.81, 1.524), 0.762, new Rotation2d(), Rotation2d.fromDegrees(-180), true),
         new CirclePath(new Translation2d(6.096, 3.048), 0.762, new Rotation2d(), Rotation2d.fromDegrees(180), false),
         new CirclePath(new Translation2d(7.62, 1.524), 0.762, Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(90), false),
         new Pose2d(3.81, 2.286, Rotation2d.fromDegrees(180)), new Pose2d(1.067, 2.286, Rotation2d.fromDegrees(180))
        ),  
        0.0, false, false);  
        
    addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(0.763, 2.286, new Rotation2d()))), mp,
        new InstantCommand(() -> driveTrain.stop()));
    */

    // two changes: ending x is at 42, not the starting 30,  ending velocity is 0, not their max speed 120 inches/s or 3 m/s
    mp = new NewRunMotionProfile(driveTrain, odometry, 0.0,
    List.of(new Pose2d(Units.inchesToMeters(30.0), Units.inchesToMeters(90.0), new Rotation2d()),
        new CirclePath(new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(60)), Units.inchesToMeters(30), new Rotation2d(), Rotation2d.fromDegrees(-180), true),
        new CirclePath(new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(120)), Units.inchesToMeters(30), new Rotation2d(), Rotation2d.fromDegrees(180), false),
        new CirclePath(new Translation2d(Units.inchesToMeters(300), Units.inchesToMeters(60)), Units.inchesToMeters(30), Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(90),
            false),
        new Pose2d(Units.inchesToMeters(150.0), Units.inchesToMeters(90), Rotation2d.fromDegrees(180)), new Pose2d(Units.inchesToMeters(30.0), Units.inchesToMeters(90.0), Rotation2d.fromDegrees(180))),
    0.0, false, false);
    // Add your addCommands(new FooCommand(), new BarCommand());
    //addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d()))), mp,
       // new InstantCommand(() -> driveTrain.stop()));

        addCommands(
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new InstantCommand(() -> odometry.setPosition(new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d()))), mp,
                    new InstantCommand(() -> driveTrain.stop())
                )
                , 
            new StartStopTimer())
        );
    
  }


  public static void main(String[] args) {

    RunAutoNavBarrelRacing cmd = new RunAutoNavBarrelRacing(null, null);
    //cmd.mp.visualize(80, List.of());
    cmd.mp.visualize(80, 
     List.of(new Translation2d( Units.inchesToMeters(30),  Units.inchesToMeters(120)), new Translation2d( Units.inchesToMeters(60),  Units.inchesToMeters(120)), new
     Translation2d( Units.inchesToMeters(30),  Units.inchesToMeters(60)),
     new Translation2d( Units.inchesToMeters(60),  Units.inchesToMeters(60)), new Translation2d( Units.inchesToMeters(150),  Units.inchesToMeters(60)), new Translation2d( Units.inchesToMeters(240),
     Units.inchesToMeters(120)),
     new Translation2d( Units.inchesToMeters(300),  Units.inchesToMeters(60))
     ) 
     );
  }

}
