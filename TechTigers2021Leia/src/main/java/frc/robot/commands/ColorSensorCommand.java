/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;

public class ColorSensorCommand extends CommandBase {
  private int RainbowColor;
  private double startTimeColorSensor = 0;
  private int DesiredRainbowColor;
  
  public ColorSensorCommand() {
    addRequirements(RobotContainer.colorWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.colorWheel.zeroSensor();
    startTimeColorSensor = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     RainbowColor = RobotContainer.colorWheel.getRainbow();
     DesiredRainbowColor = RobotContainer.colorWheel.colorTransfer(RobotContainer.colorWheel.colorMatching());
     //Blue = 1 Red = 2 Green = 3 Yellow = 4
    if (!(RainbowColor == DesiredRainbowColor)){
      RobotContainer.colorWheel.manualColorWheelSpin(0.1);
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.colorWheel.zeroSensor();
  }

  @Override
  public boolean isFinished() {
    if (RainbowColor == DesiredRainbowColor){
      return true;
    }
    if(startTimeColorSensor + RobotMap.colorSensorTimeout < Timer.getFPGATimestamp()) {
       return true;
    }
    return false;
  }
}
