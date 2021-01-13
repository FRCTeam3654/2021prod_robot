/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
//import frc.robot.RobotMap;
import frc.robot.RobotMap;

public class ClimbCommand extends Command {
  public ClimbCommand() {
    requires(Robot.climb);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //sets the z channel for climb on xbox joystick so it works
    Robot.oi.operatorStick.setZChannel(3);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double joystickX;
    double joystickZ;
    joystickX = (Robot.oi.operatorStick.getY());
    joystickZ = (Robot.oi.operatorStick.getZ()); 

    //joystickX = handleDeadband(joystickX, RobotMap.joystickDeadBand);
    //joystickY = handleDeadband(joystickY, RobotMap.joystickDeadBand);
    
    if (Math.abs(joystickX) < RobotMap.joystickDeadBand)
    {
      joystickX = 0;
    }
    if (Math.abs(joystickZ) < RobotMap.joystickDeadBand)
    {
      joystickZ = 0;
    }

    if (joystickX < 0)
    {
      joystickX *= 0.5;
    }
    if (joystickZ < 0)
    {
      joystickZ *= 0.5;
    }


    Robot.climb.karenaNotArcadeDrive(joystickZ, joystickX);
     
    System.out.println("ElevatorX = " + joystickX + "ElevatorZ = " + joystickZ);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
