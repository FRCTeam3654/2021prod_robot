/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ColorWheelCommand extends Command {

  public ColorWheelCommand() {
    requires(Robot.colorWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.colorWheel.zeroSensor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.colorWheel.spinColorWheel(RobotMap.colorWheelSpinTickAmount);
  }

  // Called once the command ends or is interrupted.
  @Override
  protected void end() {
    Robot.colorWheel.manualColorWheelSpin(0.0);
    Robot.colorWheel.zeroSensor();
    Robot.drive.resetToPercentAndzeroDistance();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Robot.colorWheel.colorWheelTickCount() >= RobotMap.colorWheelSpinTickAmount){
      Robot.colorWheel.zeroSensor();
      return true;
    }
    return false;
  }
}
