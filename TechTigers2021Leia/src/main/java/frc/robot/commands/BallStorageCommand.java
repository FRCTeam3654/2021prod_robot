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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

public class BallStorageCommand extends CommandBase {
  private double startTimer = 0;
  private boolean isBallMoving =false;
  boolean isBeltRunningLastCycle = false;
  double beltTimer = 0.0;

  public BallStorageCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    addRequirements(RobotContainer.ballStorage);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    boolean stgDist1=false;
    boolean stgDist2=false;
    boolean stgDist3=false;
    //boolean stgDist4=false;
    //boolean stgDist5=false;
    

    if(RobotContainer.ballStorage.storageSensor1() > 1200  && !RobotContainer.oi.ballFlushButton.get()) stgDist1=true; 
    if(RobotContainer.ballStorage.storageSensor2()  && !RobotContainer.oi.ballFlushButton.get()) stgDist2=true; 
    if(RobotContainer.ballStorage.storageSensor3()  && !RobotContainer.oi.ballFlushButton.get()) stgDist3=true; 
   // if(RobotContainer.ballStorage.storageSensor4() > 1200  && !RobotContainer.oi.ballFlushButton.get()) stgDist4=true; 
    //if(RobotContainer.ballStorage.storageSensor5() && !RobotContainer.oi.ballFlushButton.get()) stgDist5=true; 

    SmartDashboard.putBoolean("Distance Sensor 1", stgDist1);
    SmartDashboard.putNumber("Distance Sensor 1 Analog",RobotContainer.ballStorage.storageSensor1());
    SmartDashboard.putBoolean("Distance Sensor 2", stgDist2);
    SmartDashboard.putBoolean("Distance Sensor 3", stgDist3);
    //SmartDashboard.putBoolean("Distance Sensor 4", stgDist4);
    //SmartDashboard.putBoolean("Distance Sensor 5", stgDist5);

    boolean stgMot1=true;
    boolean stgMot2=true;
    //boolean stgMot3=true;
    //boolean stgMot4=true;
    
    //If 5 then motor 4 OFF
    //if(stgDist5) {
     // stgMot4 =false; 
     // if (!stgDist1 && !stgDist2 && !stgDist3 && !stgDist4){   // if No other balls shut rest down
     //   stgMot1 =false; 
     //   stgMot2 =false;
     //   stgMot3 =false;
     // }
    //}

    //If 4 & 5 then motor 3 OFF
    //if(stgDist4 && stgDist5) {
    //  stgMot3 =false; 
    //  if (!stgDist1 && !stgDist2 && !stgDist3){  // if No other balls shut rest down
    //    stgMot1 =false; 
    //    stgMot2 =false;
    //  }
    //}

    //If 3 & 4 & 5 then motor 2 OFF
    if(stgDist3 ) {  // if No other balls shut rest down
      stgMot2 =false;
      //if (!stgDist1 && !stgDist2){
        //stgMot1 =false; 
      //}
    }

    //If 2 & 3 & 4 & 5 then motor 1 OFF
    if(stgDist2 && stgDist3 ) {  // We are full
      stgMot1 =false;
    }

    //if nothing on  then all off
    if(!stgDist1 && !stgDist2 && !stgDist3 ){
      stgMot1 =false; 
      stgMot2 =false;
      //stgMot3 =false;
      //stgMot4 =false;
    }
    //if sensor 1 off then all off
    if(!stgDist1){
      stgMot1 =false; 
      stgMot2 =false;
      //stgMot3 =false;
      //stgMot4 =false;
    }

    SmartDashboard.putBoolean("Distance Motor 1", stgMot1);
    SmartDashboard.putBoolean("Distance Motor 2", stgMot2);
  //  SmartDashboard.putBoolean("Distance Motor 3", stgMot3);
  //  SmartDashboard.putBoolean("Distance Motor 4", stgMot4);

    if (!RobotContainer.oi.ballFlushButton.get()){

      if(stgMot1) { 
        
        RobotContainer.ballStorage.driveBallStorage1(RobotMap.ballStorageSpeed); 
        isBeltRunningLastCycle = true;
        beltTimer = Timer.getFPGATimestamp();
      }
      else {
        
        
        if (isBeltRunningLastCycle == true){
          //if (Timer.getFPGATimestamp() - beltTimer > 0.01){
            if (Timer.getFPGATimestamp() - beltTimer > 2 || stgMot2 == false){

            RobotContainer.ballStorage.driveBallStorage1(0);
            isBeltRunningLastCycle = false;
          }
          else 
          {
          RobotContainer.ballStorage.driveBallStorage1(RobotMap.ballStorageSpeed);
          }

        }
        else 
        {
          RobotContainer.ballStorage.driveBallStorage1(0);
        }
      
      }

      if(stgMot2)  RobotContainer.ballStorage.driveBallStorage2(RobotMap.ballStorageSpeed); else RobotContainer.ballStorage.driveBallStorage2(0); 
     // if(stgMot3)  RobotContainer.ballStorage.driveBallStorage3(RobotMap.ballStorageSpeed); else RobotContainer.ballStorage.driveBallStorage3(0); 
     // if(stgMot4)  RobotContainer.ballStorage.driveBallStorage4(RobotMap.ballStorageSpeed); else RobotContainer.ballStorage.driveBallStorage4(0); 
     }

    if (startTimer + RobotMap.ballStorageTimerAndysVision < Timer.getFPGATimestamp())  isBallMoving = false;

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

}