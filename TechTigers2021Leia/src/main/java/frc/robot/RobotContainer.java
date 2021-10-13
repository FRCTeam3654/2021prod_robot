// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
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

    autoChooser.setDefaultOption("Auto (Shot and Move)", new NewCommandGroup(odometry, drive));
    
    //autoChooser.setDefaultOption("AutoNav (Barrel Racing)", new RunAutoNavBarrelRacing(odometry, drive));
    
    autoChooser.addOption("Galactic Search (A)", new RunGalacticSearchA(odometry, drive));
    autoChooser.addOption("Galactic Search (B)", new RunGalacticSearchB(odometry, drive));

    autoChooser.addOption("AutoNav (Slalom)", new RunAutoNavSlalom(odometry, drive));
    autoChooser.addOption("AutoNav (Bounce)", new RunAutoNavBounce(odometry, drive));
    
    autoChooser.addOption("Galactic Search (A/Blue)", new RunGalacticSearchABlue(odometry, drive));
    autoChooser.addOption("Galactic Search (A/Red)", new RunGalacticSearchARed(odometry, drive));
    autoChooser.addOption("Galactic Search (B/Blue)", new RunGalacticSearchBBlue(odometry, drive));
    autoChooser.addOption("Galactic Search (B/Red)", new RunGalacticSearchBRed(odometry, drive));
    
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

}
