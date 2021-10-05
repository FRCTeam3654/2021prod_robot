/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Turn90DegreesCommand;
//import frc.robot.commands.AutonomousDriveCommand;
import frc.robot.commands.BallFlushCommand;
import frc.robot.commands.BallPickUpCommand;
import frc.robot.commands.ColorWheelCommand;
import frc.robot.commands.ColorSensorCommand;
import frc.robot.commands.BallShooterCommand;
import frc.robot.commands.DriveTargetCommand;
//import frc.robot.commands.BallFlushCommand;
import frc.robot.commands.ClimbLockCommand;
import frc.robot.commands.TurretTurningCommand;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  public Joystick driverStick = new Joystick(RobotMap.driverJoystickPort);
  public Joystick operatorStick = new Joystick(RobotMap.operatorJoystickPort);
  public JoystickButton turboButton;
  public JoystickButton driveStraightButton;
  public JoystickButton turnLeft90Button;
  public JoystickButton turnRight90Button; 
  public JoystickButton turnLeft180Button;
  public JoystickButton turnRight180Button;
  public JoystickButton lightsButton;
  public JoystickButton colorWheelSpinButton;
  public JoystickButton colorSensorButton;
  public JoystickButton ballPickUpButton;
  public JoystickButton ballShooterButton;
  public JoystickButton ballFlushButton;
  public JoystickButton limeLightButton;
  public JoystickButton climbLockLeftButton;
  public JoystickButton climbLockRightButton;
  public JoystickButton climbUnlockLeftButton;
  public JoystickButton climbUnlockRightButton;
  public JoystickButton turretButton;

  public OI(){

  turboButton = new JoystickButton(driverStick, RobotMap.turboButtonNumber);
  driveStraightButton = new JoystickButton(driverStick, RobotMap.driveStraightButtonNumber);
  turnLeft90Button = new JoystickButton(driverStick, RobotMap.turnLeft90ButtonNumber);
  turnRight90Button = new JoystickButton(driverStick, RobotMap.turnRight90ButtonNumber);
  turnLeft180Button = new JoystickButton(driverStick, RobotMap.turnLeft180ButtonNumber);
  turnRight180Button = new JoystickButton(driverStick, RobotMap.turnRight180ButtonNumber);
  colorWheelSpinButton = new JoystickButton(operatorStick, RobotMap.colorWheelSpinButtonNumber);
  colorSensorButton = new JoystickButton(operatorStick, RobotMap.colorSensorButtonNumber);
  ballPickUpButton = new JoystickButton(operatorStick, RobotMap.pickUpButtonNumber); //change to operatorStick when we have both joystick
  ballShooterButton = new JoystickButton(operatorStick, RobotMap.ballShooterButtonNumber); //change to operatorStick when we have both joystick
  ballFlushButton = new JoystickButton(operatorStick, RobotMap.ballFlushButtonNumber);
  limeLightButton = new JoystickButton(driverStick, RobotMap.limeLightButtonNumber);
  climbLockLeftButton = new JoystickButton(operatorStick, RobotMap.climbLockLeftButtonNumber);
  climbLockRightButton = new JoystickButton(operatorStick, RobotMap.climbLockRightButtonNumber);
  climbUnlockLeftButton = new JoystickButton(operatorStick, RobotMap.climbUnlockLeftButtonNumber);
  climbUnlockRightButton = new JoystickButton(operatorStick, RobotMap.climbUnlockRightButtonNumber);
  turretButton = new JoystickButton(operatorStick, RobotMap.turretButtonNumber);

  turnLeft90Button.whenPressed(new Turn90DegreesCommand());
  turnRight90Button.whenPressed(new Turn90DegreesCommand());
  turnLeft180Button.whenPressed(new Turn90DegreesCommand());
  turnRight180Button.whenPressed(new Turn90DegreesCommand());
  colorWheelSpinButton.whenPressed(new ColorWheelCommand());
  colorSensorButton.whenPressed(new ColorSensorCommand());
  ballPickUpButton.whenPressed(new BallPickUpCommand());
  ballShooterButton.whenPressed(new BallShooterCommand());
  ballFlushButton.whenPressed(new BallFlushCommand());
  limeLightButton.whenPressed(new DriveTargetCommand());
  climbLockLeftButton.whenPressed(new ClimbLockCommand());
  climbLockRightButton.whenPressed(new ClimbLockCommand());
  climbUnlockLeftButton.whenPressed(new ClimbLockCommand());
  climbUnlockRightButton.whenPressed(new ClimbLockCommand());
  turretButton.whenPressed(new TurretTurningCommand());
  }
  
}
