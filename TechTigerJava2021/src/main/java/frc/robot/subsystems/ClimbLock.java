/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ClimbCommand;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * Add your docs here.
 */
public class ClimbLock extends Subsystem {
  private Compressor climbLockCompressor = new Compressor();
  private DoubleSolenoid climbLockLeftSolenoid = new DoubleSolenoid(RobotMap.climbLockLeftSolenoidIn, RobotMap.climbLockLeftSolenoidOut);
  private DoubleSolenoid climbLockRightSolenoid = new DoubleSolenoid(RobotMap.climbLockRightSolenoidIn, RobotMap.climbLockRightSolenoidOut);
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public ClimbLock(){
    climbLockCompressor.start();
    climbLockCompressor.setClosedLoopControl(true);
    climbLockLeftSolenoid(true);
    climbLockRightSolenoid(true);
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

  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new ClimbCommand());

    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
