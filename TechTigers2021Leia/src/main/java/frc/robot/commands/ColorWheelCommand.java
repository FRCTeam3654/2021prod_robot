/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;

public class ColorWheelCommand extends CommandBase {

  public ColorWheelCommand() {
    addRequirements(RobotContainer.colorWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.colorWheel.zeroSensor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.colorWheel.spinColorWheel(RobotMap.colorWheelSpinTickAmount);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.colorWheel.manualColorWheelSpin(0.0);
    RobotContainer.colorWheel.zeroSensor();
    RobotContainer.drive.resetToPercentAndzeroDistance();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotContainer.colorWheel.colorWheelTickCount() >= RobotMap.colorWheelSpinTickAmount){
      RobotContainer.colorWheel.zeroSensor();
      return true;
    }
    return false;
  }
}
