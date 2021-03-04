// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import frc.robot.subsystems.RobotOdometry;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
//import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import frc.robot.subsystems.*;
import frc.robot.OI;
import frc.robot.commands.*;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  //private final Drive m_robotDrive = new Drive();
  public static Drive drive;
  public static Climb climb;
  public static ColorWheel colorWheel;
  public static BallPickUp ballPickUp;
  public static BallShooter ballShooter;
  public static BallStorage ballStorage;
  public static Turret turret;
  public static OI oi;

  private RobotOdometry odometry;

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // VERY IMPORTANT:   drive need be created before oi since oi creates Turn90DegreesCommand object in which need drive object
    drive = new Drive();
    climb = new Climb();
    colorWheel = new ColorWheel();
    ballPickUp = new BallPickUp();
    ballShooter = new BallShooter();
    ballStorage = new BallStorage();
    turret = new Turret();
    oi = new OI();  // need be after drive object

    // Configure the button bindings
    configureButtonBindings();

    drive.resetEncoders();
    drive.resetHeading();
    odometry = new RobotOdometry(drive, drive.getPigeonIMU());
    odometry.resetOdometry();

    drive.setDefaultCommand(new ManualDriveCommand());
    climb.setDefaultCommand(new ClimbCommand());
    ballStorage.setDefaultCommand( new BallStorageCommand());
    ballPickUp.setDefaultCommand(new BallPickUpCommand());

    autoChooser.setDefaultOption("AutoNav (Barrel Racing)", new RunAutoNavBarrelRacing(odometry, drive));

    /*autoChooser.addOption("Galactic Search (A/Blue)", new RunGalacticSearchABlue(odometry, drive));
    autoChooser.addOption("Galactic Search (A/Red)", new RunGalacticSearchARed(odometry, drive));
    autoChooser.addOption("Galactic Search (B/Blue)", new RunGalacticSearchBBlue(odometry, drive));
    autoChooser.addOption("Galactic Search (B/Red)", new RunGalacticSearchBRed(odometry, drive));
    */
    autoChooser.addOption("Galactic Search (A)", new RunGalacticSearchA(odometry, drive));
    autoChooser.addOption("Galactic Search (B)", new RunGalacticSearchB(odometry, drive));

    autoChooser.addOption("AutoNav (Slalom)", new RunAutoNavSlalom(odometry, drive));
    autoChooser.addOption("AutoNav (Bounce)", new RunAutoNavBounce(odometry, drive));
    
    SmartDashboard.putData("Auto Mode", autoChooser);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
   
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand2() {

    drive.resetEncoders();
    drive.resetHeading();
    //drive.resetOdometry();

   /*
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            //List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            //List.of(new Translation2d(1, 0), new Translation2d(1.762, -0.762), new Translation2d(1, -1.524), new Translation2d(0.238, -0.762)),
            //List.of(new Translation2d(1, 0), new Translation2d(1.762, 0.762), new Translation2d(1, 1.524)),
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(1, 1, new Rotation2d(90)), // working
           // new Pose2d(0.5, 1.524, new Rotation2d(179.99)),
            //new Pose2d(0.238, 0.762, new Rotation2d(270)),
            //new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    */
    // m_robotDrive.tankDriveVolts(2, 2) works;
    RunAutoNavBarrelRacing barrelCmd = new RunAutoNavBarrelRacing(odometry,drive);
    return barrelCmd;

     //return new InstantCommand(() -> m_robotDrive.arcadeDrive(0.3, 0));
     // return new InstantCommand(() -> m_robotDrive.tankDriveVolts(2, 2));
  }
}
