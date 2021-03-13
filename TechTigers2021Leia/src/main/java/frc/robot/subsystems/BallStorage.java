/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.InvertType; 
import com.ctre.phoenix.motorcontrol.ControlMode; 
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class BallStorage extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public int ballCounter = 3;
  private AnalogInput analogDistanceSensor1;
  private DigitalInput digitalDistanceSensor2;
  private DigitalInput digitalDistanceSensor3;
 // private AnalogInput analogDistanceSensor4;
  //private DigitalInput digitalDistanceSensor5;

  private TalonSRX ballStorageTalon1 = new TalonSRX (RobotMap.BallStorageID1); //bottom motor of belt, neg value to move belt forward
  private TalonSRX ballStorageTalon2 = new TalonSRX (RobotMap.BallStorageID2); //motor after belt, moves ball to shooter 
 // private TalonSRX ballStorageTalon3 = new TalonSRX (RobotMap.BallStorageID3);
 // private TalonSRX ballStorageTalon4 = new TalonSRX (RobotMap.BallStorageID4);
  

  public BallStorage(){
    analogDistanceSensor1 = new AnalogInput(RobotMap.analogDistanceSensorPort1);
    digitalDistanceSensor2 = new DigitalInput(RobotMap.digitalDistanceSensorPort2);
    digitalDistanceSensor3 = new DigitalInput(RobotMap.digitalDistanceSensorPort3);
    //analogDistanceSensor4 = new AnalogInput(RobotMap.analogDistanceSensorPort4);
    //digitalDistanceSensor5 = new DigitalInput(RobotMap.digitalDistanceSensorPort5);

    analogDistanceSensor1.setAverageBits(40);
    /*analogDistanceSensor2.setAverageBits(40);
    analogDistanceSensor3.setAverageBits(40);
    analogDistanceSensor4.setAverageBits(40);
    */

    ballStorageTalon1.configFactoryDefault();
    ballStorageTalon2.configFactoryDefault();
    //ballStorageTalon3.configFactoryDefault();
    //ballStorageTalon4.configFactoryDefault();
    

    ballStorageTalon1.setNeutralMode(NeutralMode.Brake);
    ballStorageTalon2.setNeutralMode(NeutralMode.Brake);
    //ballStorageTalon3.setNeutralMode(NeutralMode.Brake);
    //ballStorageTalon4.setNeutralMode(NeutralMode.Brake);
    
  }

  public double storageSensor1() {
    //double cmDistanceSensor = (27048/(analogDistanceSensor.getAverageValue()-36))-4;
    double cmDistanceSensor1;
    cmDistanceSensor1 = analogDistanceSensor1.getAverageValue();
    SmartDashboard.putNumber("Analog Distance Sensor 1 raw", cmDistanceSensor1);
    return cmDistanceSensor1;
  }

  public boolean storageSensor2() {
    //double cmDistanceSensor = (27048/(analogDistanceSensor.getAverageValue()-36))-4;
    boolean cmDistanceSensor2;
    cmDistanceSensor2 = !digitalDistanceSensor2.get();
    SmartDashboard.putBoolean("Digital Distance Sensor 2 raw", cmDistanceSensor2);
    return cmDistanceSensor2;
  }

  public boolean storageSensor3() {
    //double cmDistanceSensor = (27048/(analogDistanceSensor.getAverageValue()-36))-4;
    boolean cmDistanceSensor3;
    cmDistanceSensor3 = !digitalDistanceSensor3.get();
    SmartDashboard.putBoolean("Digital Distance Sensor 3 raw", cmDistanceSensor3);
    return cmDistanceSensor3;
  }
/*
  public double storageSensor4() {
    //double cmDistanceSensor = (27048/(analogDistanceSensor.getAverageValue()-36))-4;
    double cmDistanceSensor4;
    cmDistanceSensor4 = analogDistanceSensor4.getAverageValue();
    SmartDashboard.putNumber("Analog Distance Sensor 4 raw", cmDistanceSensor4);
    return cmDistanceSensor4;
  }

  public boolean storageSensor5() {
    //double cmDistanceSensor = (27048/(analogDistanceSensor.getAverageValue()-36))-4;
    boolean cmDistanceSensor5;
    cmDistanceSensor5 = !digitalDistanceSensor5.get(); //sensor inverted
    SmartDashboard.putBoolean("Digital Distance Sensor 5 raw", cmDistanceSensor5);
    return cmDistanceSensor5;
  }
*/
  public void driveBallStorage1(double percentOutput){
    // if (Robot.oi.reverseBallShooterButton.get()){
    //    Robot.ballStorage.driveBallStorage((-1) * percentOutput); // reverse the direction to clear the jam
    // }
    ballStorageTalon1.set(ControlMode.PercentOutput, percentOutput);
    SmartDashboard.putNumber("Ball Motor 1 Speed", percentOutput);
  }
  
  public void driveBallStorage2(double percentOutput){
    // if (Robot.oi.reverseBallShooterButton.get()){
    //    Robot.ballStorage.driveBallStorage((-1) * percentOutput); // reverse the direction to clear the jam
    // }
    ballStorageTalon2.set(ControlMode.PercentOutput, percentOutput);
    SmartDashboard.putNumber("Ball Motor 2 Speed", percentOutput);
  }
  /*
  public void driveBallStorage3(double percentOutput){
    // if (Robot.oi.reverseBallShooterButton.get()){
    //    Robot.ballStorage.driveBallStorage((-1) * percentOutput); // reverse the direction to clear the jam
    // }
    ballStorageTalon3.set(ControlMode.PercentOutput, percentOutput);
    SmartDashboard.putNumber("Ball Motor 3 Speed", percentOutput);
  }
  
  public void driveBallStorage4(double percentOutput){
    // if (Robot.oi.reverseBallShooterButton.get()){
    //    Robot.ballStorage.driveBallStorage((-1) * percentOutput); // reverse the direction to clear the jam
    // }
    ballStorageTalon4.set(ControlMode.PercentOutput, percentOutput);
    SmartDashboard.putNumber("Ball Motor 4 Speed", percentOutput);
  }
*/

  @Override
  public void periodic() {
   
   
  }
 
}