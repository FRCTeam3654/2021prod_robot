// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.motorcontrol.InvertType; 
import com.ctre.phoenix.motorcontrol.ControlMode; 
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

/** Add your docs here. */
public class Turret extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonFX turretTurningTalon = new TalonFX (RobotMap.turretTurningID);
  private double turretTargetPosition;

  public Turret(){
    turretTurningTalon.configFactoryDefault();
    
    
    turretTurningTalon.setNeutralMode(NeutralMode.Brake);
    turretTurningTalon.configNeutralDeadband(0.01, RobotMap.pidLoopTimeout); //during testing was 0.001
    
    turretTurningTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, RobotMap.pidLoopTimeout);

    /* Set relevant frame periods to be at least as fast as periodic rate */
		turretTurningTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.pidLoopTimeout);
		turretTurningTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.pidLoopTimeout);

		/* Set the peak and nominal outputs */
		turretTurningTalon.configNominalOutputForward(0, RobotMap.pidLoopTimeout);
		turretTurningTalon.configNominalOutputReverse(0, RobotMap.pidLoopTimeout);
		turretTurningTalon.configPeakOutputForward(1, RobotMap.pidLoopTimeout);
		turretTurningTalon.configPeakOutputReverse(-1, RobotMap.pidLoopTimeout);

		/* Set gains in slot0 - see documentation */
    turretTurningTalon.selectProfileSlot(RobotMap.kShooterSlotIDx, RobotMap.kPIDLoopIDx);
		turretTurningTalon.config_kF(RobotMap.kShooterSlotIDx, RobotMap.turretGainsVelocity.kF, RobotMap.pidLoopTimeout);
		turretTurningTalon.config_kP(RobotMap.kShooterSlotIDx, RobotMap.turretGainsVelocity.kP, RobotMap.pidLoopTimeout);
		turretTurningTalon.config_kI(RobotMap.kShooterSlotIDx, RobotMap.turretGainsVelocity.kI, RobotMap.pidLoopTimeout);
		turretTurningTalon.config_kD(RobotMap.kShooterSlotIDx, RobotMap.turretGainsVelocity.kD, RobotMap.pidLoopTimeout);

		/* Set acceleration and vcruise velocity - see documentation */
		turretTurningTalon.configMotionCruiseVelocity(500, RobotMap.pidLoopTimeout);
		turretTurningTalon.configMotionAcceleration(500, RobotMap.pidLoopTimeout);

		/* Zero the sensor once on robot boot up */    
    zeroSensor();
  }
  
  public void zeroSensor() {
    turretTurningTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.kTimeoutMs);
    System.out.println("Turret Turning Sensor is 0");
  }

  public void turretTurning(double targetPos) {
    if(targetPos > 5000){
      targetPos = 5000;
    }
    if(targetPos < -5000){
      targetPos = -5000;  //adjust 5000 based on experimental limits 
    }
    turretTargetPosition = targetPos;
    turretTurningTalon.set(ControlMode.MotionMagic, targetPos);
    System.out.println(turretTurningTalon.getSelectedSensorVelocity(0) + ",  " + turretTurningTalon.getClosedLoopError(0) + ",  " + turretTurningTalon.getSelectedSensorPosition(0));
  }

  public int turretTickCount(){
    return (int)turretTurningTalon.getSelectedSensorPosition();   
  }

  public void manualTurret(double percentOutput){
    turretTurningTalon.set(ControlMode.PercentOutput, percentOutput);
  }

  public boolean atTargetPosition(){
    if(Math.abs(turretTargetPosition - (double)turretTickCount()) <= 400){ //100 //deadband for position of turret when shooting
      return true;
    }
    return false;
  }
  
  @Override
  public void periodic() {
  }
}