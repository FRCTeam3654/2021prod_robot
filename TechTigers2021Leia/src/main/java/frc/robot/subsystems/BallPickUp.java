/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.InvertType; 
import com.ctre.phoenix.motorcontrol.ControlMode; 
//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;


/**
 * Add your docs here.
 */
public class BallPickUp extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX ballPickUpTalon = new TalonSRX (RobotMap.BallPickUpID);
  private Compressor pickUpCompressor = new Compressor();
  private DoubleSolenoid pickUpSolenoid = new DoubleSolenoid(RobotMap.solenoidIn,RobotMap.solenoidOut);
  private DoubleSolenoid climbLockLeftSolenoid = new DoubleSolenoid(RobotMap.climbLockLeftSolenoidIn, RobotMap.climbLockLeftSolenoidOut);
  private DoubleSolenoid climbLockRightSolenoid = new DoubleSolenoid(RobotMap.climbLockRightSolenoidIn, RobotMap.climbLockRightSolenoidOut);
  //later put 0 and 1 into robot map

public BallPickUp(){
    pickUpCompressor.start();
    pickUpCompressor.setClosedLoopControl(true);
    climbLockLeftSolenoid(true);
    climbLockRightSolenoid(true);

   // testing voltage compensation mode -- hopefully will not brown out 
   // ballPickUpTalon.configVoltageCompSaturation(12); // "full output" will now scale to 11 Volts for all control modes when enabled.
   // ballPickUpTalon.enableVoltageCompensation(true); // turn on/off feature
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
      pickUpSolenoid.set(DoubleSolenoid.Value.kForward);
    }

  else{
    pickUpSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  }

  public void climbLockLeftSolenoid(boolean onOff){
    if (onOff){
      climbLockLeftSolenoid.set(DoubleSolenoid.Value.kForward);
    }
  else{
    climbLockLeftSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  }

  public void climbLockRightSolenoid(boolean onOff){
    if (onOff){
      climbLockRightSolenoid.set(DoubleSolenoid.Value.kForward);
    }
  else{
    climbLockRightSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  }

  public void ballPickUp(double percentOutput){
    ballPickUpTalon.set(ControlMode.PercentOutput, percentOutput);
    SmartDashboard.putNumber("BallPickUpPercentVoltage", percentOutput);
  }

  

  @Override
  public void periodic() {
   
   
  }
}
 


