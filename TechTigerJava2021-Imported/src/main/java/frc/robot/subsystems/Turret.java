// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode; 

/** Add your docs here. */
public class Turret extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonFX turretTurningTalon = new TalonFX (RobotMap.turretTurningID);

  public void zeroSensor() {
    turretTurningTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.kTimeoutMs);
    System.out.println("Turret Turning Sensor is 0");
  }

  public void turretTurning(double targetPos) {
    turretTurningTalon.set(ControlMode.MotionMagic, targetPos);
    System.out.println(turretTurningTalon.getSelectedSensorVelocity(0) + ",  " + turretTurningTalon.getClosedLoopError(0) + ",  " + turretTurningTalon.getSelectedSensorPosition(0));
  }

  public int turretTickCount(){
    return (int)turretTurningTalon.getSelectedSensorPosition();   
  }

  public void manualTurret(double percentOutput){
    turretTurningTalon.set(ControlMode.PercentOutput, percentOutput);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
