/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.InvertType; 
import com.ctre.phoenix.motorcontrol.ControlMode; 
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode; 
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import frc.robot.commands.BallPickUpCommand;

/**
 * Add your docs here.
 */
public class BallPickUp extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX ballPickUpTalon = new TalonSRX (RobotMap.BallPickUpID);
  private TalonSRX pickUpArmTalon = new TalonSRX(RobotMap.pickUpArmTalonID);
  private DigitalInput armStatus = new DigitalInput(RobotMap.ArmStatusID);
  //later put 0 and 1 into robot map

public BallPickUp(){
    //pickUpCompressor.start();
    //pickUpCompressor.setClosedLoopControl(true);
    //climbLockLeftSolenoid(true);
    //climbLockRightSolenoid(true);

   // testing voltage compensation mode -- hopefully will not brown out 
   // ballPickUpTalon.configVoltageCompSaturation(12); // "full output" will now scale to 11 Volts for all control modes when enabled.
   // ballPickUpTalon.enableVoltageCompensation(true); // turn on/off feature
    pickUpArmTalon.configFactoryDefault();
    pickUpArmTalon.setNeutralMode(NeutralMode.Brake);
    //configure sensor source for primary PID
    //colorWheelTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, RobotMap.kPIDLoopIDx, RobotMap.kTimeoutMs);
    pickUpArmTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, RobotMap.pickUpArmPIDLoopIDx, RobotMap.pickUpArmTimeoutMs);
    //set deadband to super small 0.001 (.1%)
    pickUpArmTalon.configNeutralDeadband(0.001, RobotMap.pickUpArmTimeoutMs);
    /* configure Talson SRX utput and sensor direction occordingly invert motor to
    *have green LEDs when driving Talon Forward / requesting positive utput phase sensor
    *to have positive increment when driving Talon Forward (Green LED) */
    pickUpArmTalon.setSensorPhase(false);
    //making this true or false does the same thing
    pickUpArmTalon.setInverted(true);
     /* set relevant frame periods to be at least as fast as periodic rate */
     pickUpArmTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.pickUpArmTimeoutMs);
     pickUpArmTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.pickUpArmTimeoutMs);
     /* set the peak and nominal outputs */
     pickUpArmTalon.configNominalOutputForward(0, RobotMap.pickUpArmTimeoutMs);
     pickUpArmTalon.configNominalOutputReverse(0, RobotMap.pickUpArmTimeoutMs);
     pickUpArmTalon.configPeakOutputForward(1, RobotMap.pickUpArmTimeoutMs);
     pickUpArmTalon.configPeakOutputReverse(-1, RobotMap.pickUpArmTimeoutMs);
     /* set the Motion Magic gains in slot0 - see documentation */
     pickUpArmTalon.config_kF(RobotMap.kPickUpArmSlotIDx, RobotMap.pickUpArmGains.kF, RobotMap.pickUpArmTimeoutMs);
     pickUpArmTalon.config_kP(RobotMap.kPickUpArmSlotIDx, RobotMap.pickUpArmGains.kP, RobotMap.pickUpArmTimeoutMs);
     pickUpArmTalon.config_kI(RobotMap.kPickUpArmSlotIDx, RobotMap.pickUpArmGains.kI, RobotMap.pickUpArmTimeoutMs);
     pickUpArmTalon.config_kD(RobotMap.kPickUpArmSlotIDx, RobotMap.pickUpArmGains.kD, RobotMap.pickUpArmTimeoutMs);
 
     pickUpArmTalon.selectProfileSlot(RobotMap.kPickUpArmSlotIDx, RobotMap.pickUpArmPIDLoopIDx);
     /* set acceleration and vcruise velocity - see documentation */
     //numbers should be experimentally derived once we have the color wheel system in place
     pickUpArmTalon.configMotionCruiseVelocity(RobotMap.pickUpArmCruiseVelocity, RobotMap.pickUpArmTimeoutMs);
     pickUpArmTalon.configMotionAcceleration(RobotMap.pickUpArmAcceleration, RobotMap.pickUpArmTimeoutMs);
     /* zero the sensor once on robot boot up*/
     pickUpArmTalon.setSelectedSensorPosition(0, RobotMap.pickUpArmPIDLoopIDx, RobotMap.pickUpArmTimeoutMs);
}

public void zeroSensor() {
  pickUpArmTalon.setSelectedSensorPosition(0, RobotMap.pickUpArmPIDLoopIDx, RobotMap.pickUpArmTimeoutMs);
  System.out.println("[ Pick up arm sensor is zeroed] \n");
}
public void movePickUpArm(double targetPos){
  //Robot.drive.setPositionClosedLoop(1000);
  pickUpArmTalon.set(ControlMode.MotionMagic, targetPos);
  System.out.println(pickUpArmTalon.getSelectedSensorVelocity(0) + ",  " + pickUpArmTalon.getClosedLoopError(0) + ",  " + pickUpArmTalon.getSelectedSensorPosition(0));
}

public int pickUpArmTickCount(){
  return (int)pickUpArmTalon.getSelectedSensorPosition();   
}

public boolean armStatusData(){
  return armStatus.get();   
}

public void manualPickUpArmMove(double percentOutput){
  pickUpArmTalon.set(ControlMode.PercentOutput, percentOutput);
}

public void moveArm(boolean Izzys_boolean){
  //Izzys_boolean = true;
  if (Izzys_boolean){
    pickUpSolenoid(true);
   ballPickUp(RobotMap.ballPickUpSpeed); //put in robot map
  }
  else {
    pickUpSolenoid(false);
    ballPickUp(0.0); //put in robot map
  }
}

  public void pickUpSolenoid(boolean onOff){
    if (onOff){
      movePickUpArm(RobotMap.pickUpArmSpinTickAmount) ;
    //  pickUpSolenoid.set(DoubleSolenoid.Value.kForward);
    }

  else{
    movePickUpArm(0) ;
    //pickUpSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  }


  public void ballPickUp(double percentOutput){
    ballPickUpTalon.set(ControlMode.PercentOutput, percentOutput);
    SmartDashboard.putNumber("BallPickUpPercentVoltage", percentOutput);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new BallPickUpCommand());
  }
  
}
 

