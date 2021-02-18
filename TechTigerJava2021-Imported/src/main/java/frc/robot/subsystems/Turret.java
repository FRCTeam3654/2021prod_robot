// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** Add your docs here. */
public class Turret extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonFX turretTurningTalon = new TalonFX (RobotMap.turretTurningID);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
