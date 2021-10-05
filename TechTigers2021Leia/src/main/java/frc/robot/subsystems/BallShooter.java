/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.InvertType; 
import com.ctre.phoenix.motorcontrol.ControlMode; 
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

public class BallShooter extends SubsystemBase {
  private TalonFX ballShooterTalon = new TalonFX (RobotMap.BallShooterID);
  private TalonFX ballShooterSlaveTalon = new TalonFX (RobotMap.BallShooterSlaveID);
  public BallShooter() {
    
    ballShooterTalon.configFactoryDefault();
    ballShooterSlaveTalon.configFactoryDefault();
    
    ballShooterSlaveTalon.follow(ballShooterTalon);
    ballShooterSlaveTalon.setInverted(InvertType.InvertMotorOutput);
   
    ballShooterTalon.setSensorPhase(true);
    
    ballShooterTalon.setNeutralMode(NeutralMode.Coast);
    ballShooterSlaveTalon.setNeutralMode(NeutralMode.Coast);
    ballShooterTalon.configNeutralDeadband(0.01, RobotMap.pidLoopTimeout); //during testing was 0.001
    
    ballShooterTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, RobotMap.pidLoopTimeout);


    /* Set relevant frame periods to be at least as fast as periodic rate */
		ballShooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.pidLoopTimeout);
		//ballShooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.pidLoopTimeout);

		/* Set the peak and nominal outputs */
		ballShooterTalon.configNominalOutputForward(0, RobotMap.pidLoopTimeout);
		ballShooterTalon.configNominalOutputReverse(0, RobotMap.pidLoopTimeout);
		ballShooterTalon.configPeakOutputForward(1, RobotMap.pidLoopTimeout);
		ballShooterTalon.configPeakOutputReverse(-1, RobotMap.pidLoopTimeout);

		/* Set gains in slot0 - see documentation */
    ballShooterTalon.selectProfileSlot(RobotMap.kShooterSlotIDx, RobotMap.kPIDLoopIDx);
		ballShooterTalon.config_kF(RobotMap.kShooterSlotIDx, RobotMap.shooterGainsVelocity.kF, RobotMap.pidLoopTimeout);
		ballShooterTalon.config_kP(RobotMap.kShooterSlotIDx, RobotMap.shooterGainsVelocity.kP, RobotMap.pidLoopTimeout);
		ballShooterTalon.config_kI(RobotMap.kShooterSlotIDx, RobotMap.shooterGainsVelocity.kI, RobotMap.pidLoopTimeout);
		ballShooterTalon.config_kD(RobotMap.kShooterSlotIDx, RobotMap.shooterGainsVelocity.kD, RobotMap.pidLoopTimeout);

		/* Set acceleration and vcruise velocity - see documentation */
		//ballShooterTalon.configMotionCruiseVelocity(1000, RobotMap.pidLoopTimeout);
		//ballShooterTalon.configMotionAcceleration(1000, RobotMap.pidLoopTimeout);

		/* Zero the sensor once on robot boot up */
		ballShooterTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.pidLoopTimeout);
    
   
    /*ballShooterTalon.config_kF(RobotMap.kShooterSlotIDx,RobotMap.shooterGainsVelocity.kF);
    ballShooterTalon.config_kP(RobotMap.kShooterSlotIDx,RobotMap.shooterGainsVelocity.kP);
    ballShooterTalon.config_kI(RobotMap.kShooterSlotIDx,RobotMap.shooterGainsVelocity.kI);
    ballShooterTalon.config_kD(RobotMap.kShooterSlotIDx,RobotMap.shooterGainsVelocity.kD);
*/
    
    
    zeroSensors();  
  }

  void zeroSensors() {
    //ballShooterTalon.getSensorCollection().setQuadraturePosition(0, RobotMap.pidLoopTimeout);
    ballShooterTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.pidLoopTimeout);
    System.out.println("Ball Shooter Sensor is zeroed\n");
  }

  public void shoot(boolean mrWuBoolean){
    if (mrWuBoolean)
    {     
      ballShooterTalon.set(ControlMode.Velocity, RobotMap.shooterSpeed_nativeUnit);
    }
    else
    {
      ballShooterTalon.set(ControlMode.Velocity, 0);
    }
    double speed = ballShooterTalon.getSelectedSensorVelocity(RobotMap.kPIDLoopIDx);
    SmartDashboard.putNumber("SensorShooterVel", speed);
    //System.out.println(""+ speed);
  }

  public boolean targetSpeed(){
    double speed = ballShooterTalon.getSelectedSensorVelocity(RobotMap.kPIDLoopIDx);
    SmartDashboard.putNumber("Shooter Speed 2", speed);
    double speedDifferential = speed - RobotMap.shooterSpeed_nativeUnit;
    SmartDashboard.putNumber("Shooter Speed Differential", speedDifferential);
    if (Math.abs(speedDifferential) < RobotMap.shooterSpeedTolerance){
      return true;
    }
    return false;
    
  }

  @Override
  public void periodic() {
   
   
  }
  
}

