/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;

public class ManualDriveCommand extends CommandBase {
  private boolean driveStraightFlag = false;
  private double driveStraightAngle = 0;
  public ManualDriveCommand() {
    // Use requires() here to declare subsystem dependencies
    addRequirements(RobotContainer.drive);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double joystickX;
    double joystickY;
    double [] yawPitchRollArray;
    yawPitchRollArray = new double [3];
    joystickX = (RobotContainer.oi.driverStick.getX() * -1);
    joystickY = (RobotContainer.oi.driverStick.getY() * -1);
    joystickX = handleDeadband(joystickX, RobotMap.joystickDeadBand);
    joystickY = handleDeadband(joystickY, RobotMap.joystickDeadBand);
    //This is to activate turbo mode. If the button is pressed, turbo mode is on
    
    if (RobotContainer.oi.turboButton.get()){
    }
    else{
      joystickX = joystickX * RobotMap.nonTurboMultiplierTurn; 
      joystickY = joystickY * RobotMap.nonTurboMultiplierForward;
    }
    RobotContainer.drive.pigeonVinnie.getYawPitchRoll(yawPitchRollArray);
    if (RobotContainer.oi.driveStraightButton.get()){  
      //joystickX = 0;
      if (!driveStraightFlag){
        driveStraightAngle = yawPitchRollArray[0];
        driveStraightFlag = true;
      }
      double vinniesError = driveStraightAngle - yawPitchRollArray[0];
      joystickX = vinniesError * RobotMap.driveStraightProportion;
    }

    else {
      driveStraightFlag = false;
    }

    //System.out.println("X=" + joystickX + "Y=" + joystickY);
    RobotContainer.drive.setArcade(joystickX, joystickY);

    //Dashboard features for Joystick x and y values and right and left encoders
    SmartDashboard.putNumber("Joystick X: ", joystickX);
    SmartDashboard.putNumber("Joystick Y: ", joystickY);
    SmartDashboard.putNumber("Left Encoder", RobotContainer.drive.leftFrontTalon.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Encoder", RobotContainer.drive.rightFrontTalon.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Yaw: ", yawPitchRollArray[0]);
  }

  //Deadband makes the center of the joystick have leeway on absolute 0
  public double handleDeadband(double val, double deadband){
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }
  
  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }


}