/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Timer;

public class ColorSensorCommand extends Command {
  private int RainbowColor;
  private double startTimeColorSensor = 0;
  private int DesiredRainbowColor;
  
  public ColorSensorCommand() {
    requires(Robot.colorWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.colorWheel.zeroSensor();
    startTimeColorSensor = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     RainbowColor = Robot.colorWheel.getRainbow();
     DesiredRainbowColor = Robot.colorWheel.colorTransfer(Robot.colorWheel.colorMatching());
     //Blue = 1 Red = 2 Green = 3 Yellow = 4
    if (!(RainbowColor == DesiredRainbowColor)){
      Robot.colorWheel.manualColorWheelSpin(0.1);
    }
  }

  @Override
  protected void end() {
    Robot.colorWheel.zeroSensor();
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
