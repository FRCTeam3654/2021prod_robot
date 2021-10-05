// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
//import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;

public class TurnSmallDegreeCommand extends CommandBase {

  double _angleToTurn = 0.0; // in Degree

  // use MotionMagic to turn small angle, so copy most code from MotionMagicCommand
  private double _distanceInMeters=0;
  private boolean _resetToPercentMode = true;
  private double startTimeAutonomous = 0;
  private boolean motionMagicAutonomousFlag = false;
  private double startTimeEndStage = 0;
  private boolean motionMagicEndStageFlag = false;

  public TurnSmallDegreeCommand() {;
    addRequirements(RobotContainer.drive);
  }

  public TurnSmallDegreeCommand(double angleToTurn, boolean resetToPercentMode) {
    addRequirements(RobotContainer.drive);
    _angleToTurn = angleToTurn;
    _resetToPercentMode  = resetToPercentMode ;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {}

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (!motionMagicAutonomousFlag){
      startTimeAutonomous = Timer.getFPGATimestamp();
      motionMagicAutonomousFlag = true;

      _distanceInMeters  = Math.abs(3.14156 * RobotMap.effectiveTurnWheelWidth * _angleToTurn / 360.0);
     
      //setOppositeTurn(true);
      //Robot.drive.setRightTalonFXInvert(false);
      RobotContainer.drive.setLeftTalonFXInvert(true);
      
      RobotContainer.drive.setMotionMagic(_distanceInMeters, 1000, 1000); 
      
    }

  }
 /*
  public void setOppositeTurn(boolean oppositeTurn) {
        if( oppositeTurn == true) {
          if(_angleToTurn > 0 )
          {
            Robot.drive.setRightTalonFXInvert(true);
            Robot.drive.setLeftTalonFXInvert(true);
          }
          else {
            // need flip the right FX from true to false, after done, restore back
            Robot.drive.setRightTalonFXInvert(false);
            Robot.drive.setLeftTalonFXInvert(false);
          }
        }
        else {
          
            Robot.drive.setRightTalonFXInvert(true);
            Robot.drive.setLeftTalonFXInvert(false);
        }
  }
  */
  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if(motionMagicAutonomousFlag == true) {
      if( (startTimeAutonomous + 0.1) > Timer.getFPGATimestamp()) {
          // if autonomous started, let it run at least 100 ms
          return false;
      } else if(RobotContainer.drive.isMotionMagicDone(_distanceInMeters * RobotMap.kMeterToFalconSenorUnit, _resetToPercentMode)){
        motionMagicAutonomousFlag = false; 
        //setOppositeTurn(false); // must restore back
        RobotContainer.drive.setLeftTalonFXInvert(false);
        return true;
      }
      // enforce timeout in case MP is stucked or running too long
      else if(startTimeAutonomous + RobotMap.motionMagicTimeOut < Timer.getFPGATimestamp()) {
          motionMagicAutonomousFlag = false;  
          if( _resetToPercentMode == true) {
             RobotContainer.drive.resetToPercentAndzeroDistance();
          }
          //setOppositeTurn(false); // must restore back
          RobotContainer.drive.setLeftTalonFXInvert(false);
           return true;
      }
      else {
          // if reach here, drive code thinks MM is not done, not reach overal time out yet, but robot may not move at all if PID is not correctly set
          double[] sensRawReading = RobotContainer.drive.getTalonSensorRawReading();
          double targetRawDistance = _distanceInMeters * RobotMap.kMeterToFalconSenorUnit;
          double leftErrorPercent = Math.abs(100.0 * (targetRawDistance - sensRawReading[0])/ targetRawDistance);
          double rightErrorPercent = Math.abs(100.0 * (targetRawDistance - sensRawReading[2])/ targetRawDistance);

          // drive can determine if MM is done with a fixed error percent, here we can define our owne error percent 
          // when position error is < 1% and raw velocity reading is < 100,  consider it is near the end stage
          if ( (leftErrorPercent < 1.0 &&  Math.abs(sensRawReading[1]) < 100) || (rightErrorPercent < 1.0 &&  Math.abs(sensRawReading[3]) < 100)  ) {
            // robot near the target, essentially it is not moving, let it doing this for at most 1 extra second
              if( motionMagicEndStageFlag == false) {
                 startTimeEndStage = Timer.getFPGATimestamp();
                 motionMagicEndStageFlag = true;
              }
              else {
                 // motionMagicEndStageFlag == true now
                 if(startTimeEndStage + 1.0 < Timer.getFPGATimestamp()) {
                    // if the robot is stuck at the end stage for more than 1 second, end the motion magic process
                    motionMagicAutonomousFlag = false;  
                    motionMagicEndStageFlag = false;
                    if( _resetToPercentMode == true) {
                      RobotContainer.drive.resetToPercentAndzeroDistance();
                    }
                    //setOppositeTurn(false); // must restore back
                    RobotContainer.drive.setLeftTalonFXInvert(false);
                    return true;
                 }
              }
          }
      }     
    }

    RobotContainer.drive.getRemote1SensorReading();
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {}

  
}
