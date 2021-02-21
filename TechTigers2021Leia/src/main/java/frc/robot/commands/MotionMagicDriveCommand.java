/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Timer;

public class MotionMagicDriveCommand extends Command {
  private double _distanceInMeters=0; 
  private boolean _resetToPercentMode = true;
  private boolean _useAuxPID = false;
  private double startTimeAutonomous = 0;
  private boolean motionMagicAutonomousFlag = false;
  private double startTimeEndStage = 0;
  private boolean motionMagicEndStageFlag = false;

  public MotionMagicDriveCommand(double distanceInMeters) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drive);
    _distanceInMeters = distanceInMeters;
    _resetToPercentMode = true; 
  }

  public MotionMagicDriveCommand(double distanceInMeters, boolean resetToPercentMode) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drive);
    _distanceInMeters = distanceInMeters;
    _resetToPercentMode  = resetToPercentMode ;
  }

  public MotionMagicDriveCommand(double distanceInMeters, boolean resetToPercentMode, boolean useAuxPID) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drive);
    _distanceInMeters = distanceInMeters;
    _useAuxPID = useAuxPID;
    _resetToPercentMode  = resetToPercentMode ;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double distanceInNativeUnit = _distanceInMeters * RobotMap.kMeterToFalconSenorUnit; 
    //Robot.drive.setMotionMagic(distanceInNativeUnit, 8000, 4000);

    if (!motionMagicAutonomousFlag){
      startTimeAutonomous = Timer.getFPGATimestamp();
      motionMagicAutonomousFlag = true;
      if( _useAuxPID == false ) {
        Robot.drive.setMotionMagic(distanceInNativeUnit, 4000, 4000); 
      }
      else {
        Robot.drive.configureArcFXDrive(false);
        
        Robot.drive.setMotionMagic(distanceInNativeUnit, Robot.drive.getRemote1SensorReading(), 4000,  4000, true);
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(motionMagicAutonomousFlag == true) {
      if( (startTimeAutonomous + 0.1) > Timer.getFPGATimestamp()) {
          // if autonomous started, let it run at least 100 ms
          return false;
      } else if(Robot.drive.isMotionMagicDone(_distanceInMeters * RobotMap.kMeterToFalconSenorUnit, _resetToPercentMode)){
        motionMagicAutonomousFlag = false; 
        return true;
      }
      // enforce timeout in case MP is stucked or running too long
      else if(startTimeAutonomous + RobotMap.motionMagicTimeOut < Timer.getFPGATimestamp()) {
          motionMagicAutonomousFlag = false;  
          if( _resetToPercentMode == true) {
             Robot.drive.resetToPercentAndzeroDistance();
          }
           return true;
      }
      else {
          // if reach here, drive code thinks MM is not done, not reach overal time out yet, but robot may not move at all if PID is not correctly set
          double[] sensRawReading = Robot.drive.getTalonSensorRawReading();
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
                      Robot.drive.resetToPercentAndzeroDistance();
                    }
                    return true;
                 }
              }
          }
      }     
    }

    Robot.drive.getRemote1SensorReading();
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
