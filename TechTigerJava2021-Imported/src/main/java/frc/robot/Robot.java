/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.       Mercy Izzy was here                                     */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Climb;
import frc.robot.commands.AutonomousDriveCommand;
import frc.robot.commands.AutonomousCommandGroup;
import frc.robot.subsystems.ColorWheel;
import frc.robot.subsystems.BallPickUp;
import frc.robot.subsystems.ClimbLock;
import frc.robot.subsystems.BallShooter;
import frc.robot.subsystems.BallStorage;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI oi;
  public static Drive drive;
  public static Climb climb;
  public static ColorWheel colorWheel;
  public static BallPickUp ballPickUp;
  public static BallShooter ballShooter;
  public static BallStorage ballStorage;
  public static ClimbLock climbLock;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code. 
   */
  @Override
  public void robotInit() {
    RobotMap.getPreference();
    // VERY IMPORTANT:   drive need be created before oi since oi creates Turn90DegreesCommand object in which need drive object
    drive = new Drive();
    climb = new Climb();
    colorWheel = new ColorWheel();
    ballPickUp = new BallPickUp();
    ballShooter = new BallShooter();
    ballStorage = new BallStorage();
    climbLock = new ClimbLock();
    oi = new OI();  // need be after drive object

    m_chooser.setDefaultOption("Mercy Auto", new AutonomousCommandGroup());
    //m_chooser.setDefaultOption("Mercy Auto", new AutonomousDriveCommand());
    //chooser.addOption("Mercy Auto", new AutonomousDriveCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    
    CameraServer.getInstance().startAutomaticCapture(0);
   // CameraServer.getInstance().startAutomaticCapture(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //3 is force off
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
   // m_autonomousCommand =     new AutonomousDriveCommand();
    m_autonomousCommand = m_chooser.getSelected();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    Robot.drive.configureDrive();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
   //double MaxClimbPercentOutput = SmartDashboard.getNumber("MaxClimbPercentOutput", 0.8);
   //System.out.println("MaxClimbPercentOutput = " + MaxClimbPercentOutput);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}