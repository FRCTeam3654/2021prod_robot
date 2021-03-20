/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motorcontrol.InvertType; 
import com.ctre.phoenix.motorcontrol.ControlMode; 
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import java.io.*;
import java.util.ArrayList;
import com.ctre.phoenix.motion.*;
import java.util.concurrent.atomic.AtomicInteger;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.controller.PIDController;

/**
 * Add your docs here.
 */
public class Drive extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands. heehee

  public DifferentialDrive differentialDrive;
  public double leftSpeed; 
  public double rightSpeed;

  public WPI_TalonFX leftFrontTalon = new WPI_TalonFX(RobotMap.leftTalonMaster);
  public WPI_TalonFX leftBackTalon = new WPI_TalonFX(RobotMap.leftTalonSlave);
  public WPI_TalonFX rightFrontTalon = new WPI_TalonFX(RobotMap.rightTalonMaster);
  public WPI_TalonFX rightBackTalon = new WPI_TalonFX(RobotMap.rightTalonSlave);
  

  public TalonSRX vinnieTalon = new TalonSRX(RobotMap.vinnieTalonNumber);

  public PigeonIMU pigeonVinnie = new PigeonIMU(vinnieTalon);

  // ramsete related variables //
  DifferentialDrive m_drive ;

  PIDController leftPIDController = new PIDController(Constants.DriveConstants.kPDriveVel, 0.0, 0.0);

  PIDController rightPIDController = new PIDController(Constants.DriveConstants.kPDriveVel, 0.0, 0.0);

  //Pose2d pose;
  //////// end of Ramsete related function /////////////////////

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  public double Kp = -0.1;
  public double min_command = 0.05;
 
  double _pigeonRemoteSensoreScaleFactor = (((double) (3600)) / 8192)  ; // the remote sensor reading is 0.1 degree each value


  BufferedTrajectoryPointStream _leftBufferedStream1 = new BufferedTrajectoryPointStream();
  BufferedTrajectoryPointStream _rightBufferedStream1 = new BufferedTrajectoryPointStream();

  BufferedTrajectoryPointStream _leftBufferedStream2 = new BufferedTrajectoryPointStream();
  BufferedTrajectoryPointStream _rightBufferedStream2 = new BufferedTrajectoryPointStream();

  BufferedTrajectoryPointStream _leftBufferedStream3 = new BufferedTrajectoryPointStream();
  BufferedTrajectoryPointStream _rightBufferedStream3 = new BufferedTrajectoryPointStream();

  BufferedTrajectoryPointStream _leftBufferedStream4 = new BufferedTrajectoryPointStream();
  BufferedTrajectoryPointStream _rightBufferedStream4 = new BufferedTrajectoryPointStream();

  TalonFXConfiguration _leftMasterconfig = new TalonFXConfiguration(); // factory default settings
  TalonFXConfiguration _rightMasterconfig = new TalonFXConfiguration();

  // very simple state machine to prevent calling set() while firing MP. 
  AtomicInteger _state = new AtomicInteger(0);

  public double [] yawPitchRollArrayStarting = null;

  @Override
  public void periodic() {
   
   
  }

   public Drive() {
    
    //readMPFile(false ); // false ==> not use Arc or pigeon
    readMPFile(RobotMap.kUseMotionProfileArc); 

    configureDrive();

  }

  
  public void configureDrive() {
      leftFrontTalon.configFactoryDefault();
      leftBackTalon.configFactoryDefault();
      rightFrontTalon.configFactoryDefault();
      rightBackTalon.configFactoryDefault();
      pigeonVinnie.configFactoryDefault();
      pigeonVinnie.setYaw(0.0);
      pigeonVinnie.setFusedHeading(0.0);
      
      leftFrontTalon.set(ControlMode.PercentOutput, 0);
      rightFrontTalon.set(ControlMode.PercentOutput, 0);
      
      leftFrontTalon.configNeutralDeadband(0.05,30);
      rightFrontTalon.configNeutralDeadband(0.05,30);
      leftBackTalon.configNeutralDeadband(0.0,30);
      rightBackTalon.configNeutralDeadband(0.0,30);
      
      leftBackTalon.follow(leftFrontTalon);
      rightBackTalon.follow(rightFrontTalon);

      leftFrontTalon.configClosedloopRamp(1);
      rightFrontTalon.configClosedloopRamp(1);
      rightFrontTalon.configOpenloopRamp(1);
      leftFrontTalon.configOpenloopRamp(1);
      
      rightFrontTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, RobotMap.pidLoopTimeout);
      leftFrontTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, RobotMap.pidLoopTimeout);

      leftFrontTalon.selectProfileSlot(0,0);
      rightFrontTalon.selectProfileSlot(0,0);

      leftFrontTalon.config_kF(0,0.045,30);// 0.045
      leftFrontTalon.config_kP(0,0.049,30); //0.095 //0.049
      leftFrontTalon.config_kI(0,0,30);
      leftFrontTalon.config_kD(0,0,30);

      rightFrontTalon.config_kF(0,0.045,30); // 0.045
      rightFrontTalon.config_kP(0,0.049,30); //0.095 //0.049
      rightFrontTalon.config_kI(0,0,30);
      rightFrontTalon.config_kD(0,0,30);
      
      leftFrontTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
      leftFrontTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);

      rightFrontTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
      rightFrontTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);

      leftFrontTalon.setInverted(false);
      rightFrontTalon.setInverted(true);

      leftBackTalon.setInverted(InvertType.FollowMaster);
      rightBackTalon.setInverted(InvertType.FollowMaster);

      rightFrontTalon.setSensorPhase(false);
      leftFrontTalon.setSensorPhase(false);
      rightBackTalon.setSensorPhase(false);
      leftBackTalon.setSensorPhase(false);

      rightFrontTalon.setNeutralMode(NeutralMode.Brake);
      leftFrontTalon.setNeutralMode(NeutralMode.Brake);
      rightBackTalon.setNeutralMode(NeutralMode.Brake);
      leftBackTalon.setNeutralMode(NeutralMode.Brake);
      
      leftFrontTalon.configNominalOutputForward(0, 30);
      leftFrontTalon.configNominalOutputReverse(0, 30);
      leftFrontTalon.configPeakOutputForward(1, 30);
      leftFrontTalon.configPeakOutputReverse(-1, 30);

      rightFrontTalon.configNominalOutputForward(0, 30);
      rightFrontTalon.configNominalOutputReverse(0, 30);
      rightFrontTalon.configPeakOutputForward(1, 30);
      rightFrontTalon.configPeakOutputReverse(-1, 30);
    
      leftFrontTalon.configMotionCruiseVelocity(8000, 30);
      leftFrontTalon.configMotionAcceleration(8000, 30);

      rightFrontTalon.configMotionCruiseVelocity(8000, 30);
      rightFrontTalon.configMotionAcceleration(8000, 30);

      leftFrontTalon.selectProfileSlot(RobotMap.kSlotIDx, RobotMap.kPIDLoopIDx);
      rightFrontTalon.selectProfileSlot(RobotMap.kSlotIDx, RobotMap.kPIDLoopIDx);

      zeroSensors();
  }


 
  public double[] getTalonSensorRawReading() {
    // return all four sensor reading
      double[] ret = new double[4];
      
      ret[0] = leftFrontTalon.getSelectedSensorPosition(0);
      ret[1] = leftFrontTalon.getSelectedSensorVelocity(0);      
      ret[2] = rightFrontTalon.getSelectedSensorPosition(0);
      ret[3] = rightFrontTalon.getSelectedSensorVelocity(0);
        
      return ret;
  }

  public double getRemote1SensorReading(){
    double sensor1Position = (leftFrontTalon.getSelectedSensorPosition(1) + rightFrontTalon.getSelectedSensorPosition(1))/2.0;
    
    SmartDashboard.putNumber("Sensor1pigenPos", sensor1Position);
    return sensor1Position;
  }

  public void setLeftTalonFXInvert(boolean invertDirection) {
    leftFrontTalon.setInverted(invertDirection);
  }

  public void setPower(double leftPower, double rightPower) {
      leftFrontTalon.set(ControlMode.PercentOutput, leftPower);
      rightFrontTalon.set(ControlMode.PercentOutput, rightPower);
  }

  /** Zero Quadrature Encoders on Talons */
	public void zeroSensors() {
			leftFrontTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.pidLoopTimeout);
      rightFrontTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.pidLoopTimeout);
      pigeonVinnie.setYaw(0, RobotMap.pidLoopTimeout);
		  pigeonVinnie.setFusedHeading(0, RobotMap.pidLoopTimeout);
      pigeonVinnie.setAccumZAngle(0, RobotMap.pidLoopTimeout);
		
			System.out.println("[Quadrature Encoders] All drive sensors are zeroed.\n");
  }
  
 public void resetToPercentAndzeroDistance(){
		  leftFrontTalon.getSensorCollection().setIntegratedSensorPosition(0,  RobotMap.pidLoopTimeout);
      rightFrontTalon.getSensorCollection().setIntegratedSensorPosition(0,  RobotMap.pidLoopTimeout);
      leftFrontTalon.set(ControlMode.PercentOutput, 0);
      rightFrontTalon.set(ControlMode.PercentOutput, 0);
  }
  
  public int lowGoalDistanceSensorValue() {
    int cmDistanceSensor = 0;
    return cmDistanceSensor;
  }

  public double[] getYawPitchRoll() {
    double [] yawPitchRollArray = null;
    if (pigeonVinnie.getState() == PigeonIMU.PigeonState.Ready) {
      yawPitchRollArray = new double [3];
      pigeonVinnie.getYawPitchRoll(yawPitchRollArray);

      if ( yawPitchRollArrayStarting == null ) {
          yawPitchRollArrayStarting  = yawPitchRollArray;
      }
    }

    return  yawPitchRollArray;

  }
  
//////  a set of functions for Ramsete ///////////////////////////

public double getRotationsLeft() {
  return (double) leftFrontTalon.getSelectedSensorPosition() * Constants.DriveConstants.gearRatio / Constants.DriveConstants.encoderTicksPerRev;
}


public double getRotationsRight() {
  return (double) rightFrontTalon.getSelectedSensorPosition() * Constants.DriveConstants.gearRatio / Constants.DriveConstants.encoderTicksPerRev;
}

public double getDistanceRight() {
  return Constants.DriveConstants.kWheelCircumferenceMeter * getRotationsRight();
}


public double getDistanceLeft() {
  return Constants.DriveConstants.kWheelCircumferenceMeter * getRotationsLeft();
}

public double getVelocityRight() {
  return Constants.DriveConstants.kWheelCircumferenceMeter * getRotationsRight() * 10;
}

public double getVelocityLeft() {
  return Constants.DriveConstants.kWheelCircumferenceMeter * getRotationsLeft() * 10;
}

public double getAverageEncoderDistance() {
  return (leftFrontTalon.getSelectedSensorPosition(0) + rightFrontTalon.getSelectedSensorPosition(0)) / 2.0;
}

public void setMaxOutput(double maxOutput) {
  m_drive.setMaxOutput(maxOutput);
}


/** Zeroes the heading of the robot. */
public void zeroHeading() {
  pigeonVinnie.setYaw(0);
}

// IEEERemainder = dividend - (divisor * Math.Round(dividend / divisor))  
// IEEEremainder(double dividend, double divisor)
//  f1 – f2 x n, where n is the mathematical integer closest to the exact mathematical value of the quotient f1/f2, and if two mathematical integers are equally close to f1/f2, then n is the integer that is even.
public double getYaw() {
  double ypr[] = {0,0,0};
  pigeonVinnie.getYawPitchRoll(ypr);
  //return Math.IEEEremainder(ypr[0], 360.0d);
  return ypr[0];
}

 //You can replace Rotation2D with getting the Yaw from the NavX and making sure it has the proper sign (pretty sure it needs to be inverted as NavX is CW positive).
// pretty sure Pigeon has CCW postive, no need negate
public Rotation2d getHeading() {
  double ypr[] = {0,0,0};
  pigeonVinnie.getYawPitchRoll(ypr);

  return Rotation2d.fromDegrees(ypr[0]);
}

public PIDController getLeftPIDController() {
  return leftPIDController;
}

public PIDController getRightPIDController() {
  return rightPIDController;
}

public PigeonIMU getPigeonIMU() {
    return pigeonVinnie;
}

public void set(double leftVoltage, double rightVoltage) {
  leftFrontTalon.setVoltage(leftVoltage);
  rightFrontTalon.setVoltage(rightVoltage); // remove -
  m_drive.feed();
}

public void resetEncoders() {
  leftFrontTalon.setSelectedSensorPosition(0);
  rightFrontTalon.setSelectedSensorPosition(0);
}

public void resetHeading() {
  pigeonVinnie.setYaw(0);
}

public DifferentialDriveWheelSpeeds getWheelSpeeds() {
  return new DifferentialDriveWheelSpeeds(
    leftFrontTalon.getSelectedSensorVelocity() * Constants.DriveConstants.gearRatio * 10.0 / Constants.DriveConstants.encoderTicksPerRev * Constants.DriveConstants.kWheelCircumferenceMeter,
    rightFrontTalon.getSelectedSensorVelocity() * Constants.DriveConstants.gearRatio * 10.0 / Constants.DriveConstants.encoderTicksPerRev * Constants.DriveConstants.kWheelCircumferenceMeter
  );
}

public void stop() {
  tankDriveVolts(0, 0);
}

public void arcadeDrive(double fwd, double rot) {
  m_drive.arcadeDrive(fwd, rot); // left and right drive in oppositve direction as of now
  m_drive.feed();
}

public void driveMetersPerSecond(double left, double right) {
  // drive left and right volecity in meter per second --- either velocityclosed loop or open loop
  //closed loop,  need consider gear ratio 10.71
  double targetVelocity_UnitsPer100ms_left = ((left / Constants.DriveConstants.kWheelCircumferenceMeter ) / Constants.DriveConstants.gearRatio) * Constants.DriveConstants.encoderTicksPerRev * 0.1 ; // ticks per 100 ms
  double targetVelocity_UnitsPer100ms_right = ((right / Constants.DriveConstants.kWheelCircumferenceMeter ) / Constants.DriveConstants.gearRatio) * Constants.DriveConstants.encoderTicksPerRev * 0.1 ; // ticks per 100 ms
  
  // OPTION #1:  use Talon Velocity PIDF without kS, kV and kA 
  leftFrontTalon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms_left);
  rightFrontTalon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms_right); 

}

public void tankDriveVolts(double leftVolts, double rightVolts) {
  leftFrontTalon.setVoltage(leftVolts);
  rightFrontTalon.setVoltage(rightVolts); // remove -
  m_drive.feed(); // move in the same direction both two positive voltage (forward)
}



//////  end of a set of functions for Ramsete ///////////////////////////

  public void setArcade(double velocity, double turn) {
      mercyArcadeDrive(velocity, turn);
  }

  public void setMotionMagic(double distance, int cruiseVelocity, int accelerationVelocity) {
    setMotionMagic (distance, 0.0, cruiseVelocity,  accelerationVelocity, false);
  }
  
  public void setMotionMagic(double distance, double turn_angle, int cruiseVelocity, int accelerationVelocity, boolean useAuxPID) {
    leftFrontTalon.configMotionCruiseVelocity(cruiseVelocity, RobotMap.pidLoopTimeout);
    leftFrontTalon.configMotionAcceleration(accelerationVelocity, RobotMap.pidLoopTimeout);
  
    rightFrontTalon.configMotionCruiseVelocity(cruiseVelocity, RobotMap.pidLoopTimeout);
    rightFrontTalon.configMotionAcceleration(accelerationVelocity, RobotMap.pidLoopTimeout);

    leftFrontTalon.selectProfileSlot(RobotMap.kSlotIDx, RobotMap.kPIDLoopIDx);
    rightFrontTalon.selectProfileSlot(RobotMap.kSlotIDx, RobotMap.kPIDLoopIDx);
  
    if( useAuxPID == false ) {
      leftFrontTalon.set(ControlMode.MotionMagic, distance);
      rightFrontTalon.set(ControlMode.MotionMagic, distance);
    }
    else {

    // the following are new for Arc setup
    leftFrontTalon.selectProfileSlot(RobotMap.kTurnAutonomousSlotIDx, RobotMap.PID_TURN);
    rightFrontTalon.selectProfileSlot(RobotMap.kTurnAutonomousSlotIDx, RobotMap.PID_TURN);


      rightFrontTalon.set(ControlMode.MotionMagic, distance, DemandType.AuxPID, turn_angle);		
      leftFrontTalon.set(ControlMode.MotionMagic, distance, DemandType.AuxPID, turn_angle);
    }
  }

  public boolean isMotionMagicDone(double targetDistanceInNativeUnit, boolean resetToPercentMode) {
    boolean ret = false;
    double sensorDistance = (leftFrontTalon.getSelectedSensorPosition(0) + rightFrontTalon.getSelectedSensorPosition(0))/2.0;
    double percentError = 100 * (targetDistanceInNativeUnit - sensorDistance)/targetDistanceInNativeUnit;

    // even though it is desired to achieve error < 1%, it depends on PID tuning, sometimes it is always achieable
    if (percentError < 0.3 || percentError < 0 ){
      if( resetToPercentMode == true) {
        leftFrontTalon.getSensorCollection().setIntegratedSensorPosition(0,  RobotMap.pidLoopTimeout);
        rightFrontTalon.getSensorCollection().setIntegratedSensorPosition(0,  RobotMap.pidLoopTimeout);

        leftFrontTalon.set(ControlMode.PercentOutput, 0);
        rightFrontTalon.set(ControlMode.PercentOutput, 0);
      }
      return true;
    }

    SmartDashboard.putNumber("SensorMMagicVel", leftFrontTalon.getSelectedSensorVelocity(0));
    return ret;
  }

  public void setMotionProfile(int pathNumber) {
  
    if( _state.get() != 2) {
      _state.set(2);

      leftFrontTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.pidLoopTimeout);
      rightFrontTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.pidLoopTimeout);

      if (pathNumber == 1){
          leftFrontTalon.startMotionProfile(_leftBufferedStream1, 10, ControlMode.MotionProfile);
          rightFrontTalon.startMotionProfile(_rightBufferedStream1, 10, ControlMode.MotionProfile);
      } else if (pathNumber == 2){     
          leftFrontTalon.startMotionProfile(_leftBufferedStream2, 10, ControlMode.MotionProfile);
          rightFrontTalon.startMotionProfile(_rightBufferedStream2, 10, ControlMode.MotionProfile);
      }
      else if (pathNumber == 3){
        leftFrontTalon.startMotionProfile(_leftBufferedStream3, 10, ControlMode.MotionProfile);
        rightFrontTalon.startMotionProfile(_rightBufferedStream3, 10, ControlMode.MotionProfile);
      }
    }
  }


  public void setPositionClosedLoop(double targetPosition){
    double leftNewPosition = leftFrontTalon.getSelectedSensorPosition(0) + targetPosition;
    double rightNewPosition = rightFrontTalon.getSelectedSensorPosition(0) + targetPosition;
    leftFrontTalon.set(ControlMode.Position, leftNewPosition);
    rightFrontTalon.set(ControlMode.Position, rightNewPosition);
  } 

  public int getCurrentMPState()  {
    return _state.get();
  }

  public boolean isMotionProfileDone() {
      boolean ret = false;
      if( _state.get() != 2 ) {
        ret = true;
      }
      else if (_state.get() == 2 && leftFrontTalon.isMotionProfileFinished() && rightFrontTalon.isMotionProfileFinished() ) {
        
        // not sure if these two Clear is absolutely needed, but it is needed in old API
        leftFrontTalon.clearMotionProfileTrajectories();
        rightFrontTalon.clearMotionProfileTrajectories();
       
        leftFrontTalon.clearMotionProfileHasUnderrun();
        rightFrontTalon.clearMotionProfileHasUnderrun();

        leftFrontTalon.set(ControlMode.PercentOutput, 0);
        rightFrontTalon.set(ControlMode.PercentOutput, 0);

        leftFrontTalon.selectProfileSlot(RobotMap.kSlotIDx, RobotMap.kPIDLoopIDx);
        rightFrontTalon.selectProfileSlot(RobotMap.kSlotIDx, RobotMap.kPIDLoopIDx);

        _state.set(3);
        ret = true;
        System.out.println("finished mp");
        SmartDashboard.putNumber("mp_status",12); 
      }

      SmartDashboard.putNumber("SensorMFVel", rightFrontTalon.getSelectedSensorVelocity(0));

      return ret;
  }

//Mercy Arcade Drive allows us to smoothly control the robot
public void mercyArcadeDrive(double joystickX, double joystickY) {

    double radiusPower = Math.hypot(joystickX, joystickY);
    double initAngle = Math.atan2(joystickX, joystickY);

    initAngle = initAngle + Math.PI/4;
    rightSpeed = radiusPower*Math.sin(initAngle);
    leftSpeed = radiusPower*Math.cos(initAngle);
    rightSpeed = rightSpeed*1.414;
    leftSpeed = leftSpeed*1.414;

    if (rightSpeed > 1) {
      rightSpeed = 1;
    }
    if (leftSpeed > 1) {
      leftSpeed = 1;
    }
    if (rightSpeed < -1) {
      rightSpeed = 1;
    }
    if (leftSpeed < -1) {
      leftSpeed = -1;
    }
          
    if (RobotMap.driveClosedLoopMode ) {
            //closed loop
            double targetVelocity_UnitsPer100ms_left = leftSpeed * 22000;
            double targetVelocity_UnitsPer100ms_right = rightSpeed * 22000;
            leftFrontTalon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms_left);
          rightFrontTalon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms_right);   
    }     
    else {
          //open loop
          leftFrontTalon.set(ControlMode.PercentOutput, leftSpeed);
          rightFrontTalon.set(ControlMode.PercentOutput, rightSpeed);
    }
  }


  private void readMPFile(){
    readMPFile(false);
  }

  private void readMPFile(boolean useArc){
    String path1FileName = "";
    String path2FileName = "";
    String path3FileName = "";
    String path4FileName = "";
    //String autonomous = SmartDashboard.getString("Autonomous", "Default");
    int autonomous = (int) Math.round(SmartDashboard.getNumber("Autonomous", 1.0)); 
    //autonomous = "LeftTurn";
    autonomous = 4;
    //if (autonomous.equals("DriveStraight") || autonomous.equals("Default")){
    if (autonomous == 0 ) {  //GalacticA     
      path2FileName = "/home/lvuser/GalacticA/mp_20ms_in_meter_arc_galacticA.csv";
      //path2FileName = "/home/lvuser/mp_20ms_in_meter_arc_2_2.csv";
    } else if (autonomous == 1){ //GalacticB
      path2FileName = "/home/lvuser/Barrel/mp_20ms_in_meter_arc_barrel.csv";
      //path2FileName = "/home/lvuser/GalacticB/mp_20ms_in_meter_arc_galacticB.csv";
       //path2FileName = "/home/lvuser/mp_20ms_in_meter_arc_1_2.csv";
    } else if (autonomous == 2){ //Bounce
      path2FileName = "/home/lvuser/Bounce/mp_20ms_in_meter_arc_bounce.csv";
      //path2FileName = "/home/lvuser/mp_20ms_in_meter_arc_3_2.csv";
    } else if (autonomous == 3){ //Slalom
      path2FileName = "/home/lvuser/Slalom/mp_20ms_in_meter_arc_Slalom.csv";
      //path2FileName = "/home/lvuser/mp_20ms_in_meter_arc_3_2.csv";
    } else if (autonomous == 4){ //Barrel
      path2FileName = "/home/lvuser/Barrel/mp_20ms_in_meter_arc_barrel.csv";
     // path2FileName = "/home/lvuser/mp_20ms_in_meter_arc_4_2.csv";
      //path3FileName = "/home/lvuser/mp_20ms_in_meter_arc_4_3.csv";
    } else if (autonomous == 5){ //CenterTurn
      path1FileName = "/home/lvuser/mp_20ms_in_meter_arc_5_1.csv";    
    } else if (autonomous == 6){ //RightTurn
      path1FileName = "/home/lvuser/mp_20ms_in_meter_arc_6_1.csv";
    } else if (autonomous == 7){ //Def30
      path1FileName = "/home/lvuser/mp_20ms_in_meter_arc_7_1.csv";
    }else if (autonomous == 8){ //Def90
    }

      ArrayList<String[]> arrayList1 = new ArrayList<String[]>();
      File file1 = new File(path1FileName); 
      try {
        BufferedReader br = new BufferedReader(new FileReader(file1)); 
        String st; 
        while ((st = br.readLine()) != null) { 
            //System.out.println(st); 
          String[] oneList = st.split(",");
            arrayList1.add(oneList);
          }
              int totalCnt = arrayList1.size();
                initBuffer(arrayList1, totalCnt, false, 0, _leftBufferedStream1, _rightBufferedStream1);
              initBuffer(arrayList1, totalCnt, false, 0, _leftBufferedStream1, _rightBufferedStream1);
      } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      } 
      
      if (path2FileName.length() > 10){
        ArrayList<String[]> arrayList2 = new ArrayList<String[]>();
        File file2 = new File(path2FileName); 
        try {
          BufferedReader br = new BufferedReader(new FileReader(file2)); 
          String st; 
          while ((st = br.readLine()) != null) { 
              //System.out.println(st); 
            String[] oneList = st.split(",");
              arrayList2.add(oneList);
            }
                int totalCnt = arrayList2.size();
                  initBuffer(arrayList2, totalCnt, true, 1, _leftBufferedStream2, _rightBufferedStream2);
        } catch (IOException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        } 
      }

      if (path3FileName.length() > 10){
        ArrayList<String[]> arrayList3 = new ArrayList<String[]>();
        File file3 = new File(path3FileName); 
        try {
          BufferedReader br = new BufferedReader(new FileReader(file3)); 
          String st; 
          while ((st = br.readLine()) != null) { 
              //System.out.println(st); 
            String[] oneList = st.split(",");
              arrayList3.add(oneList);
            }
                int totalCnt = arrayList3.size();
                  initBuffer(arrayList3, totalCnt, false, 1, _leftBufferedStream3, _rightBufferedStream3);
        } catch (IOException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        } 
      }

      if (path4FileName.length() > 10){
        ArrayList<String[]> arrayList4 = new ArrayList<String[]>();
        File file4 = new File(path4FileName); 
        try {
          BufferedReader br = new BufferedReader(new FileReader(file4)); 
          String st; 
          while ((st = br.readLine()) != null) { 
              //System.out.println(st); 
            String[] oneList = st.split(",");
              arrayList4.add(oneList);
            }
                int totalCnt = arrayList4.size();
                  initBuffer(arrayList4, totalCnt, true, 1, _leftBufferedStream4, _rightBufferedStream4);
                
        } catch (IOException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        } 
      }
    }

    private void initBuffer(ArrayList<String[]> profile, int totalCnt, boolean forward, int slotNumber, BufferedTrajectoryPointStream _leftBufferedStream, BufferedTrajectoryPointStream _rightBufferedStream) {
       // boolean forward = true; // set to false to drive in opposite direction of profile (not really needed
                                // since you can use negative numbers in profile).
        TrajectoryPoint leftPoint = new TrajectoryPoint(); // temp for for loop, since unused params are initialized
        TrajectoryPoint rightPoint = new TrajectoryPoint();    

        /* clear the buffer, in case it was used elsewhere */
        _leftBufferedStream.Clear();
        _rightBufferedStream.Clear();

        // MP unit is :  meter for Position, meter/second for Velocity ===> need convert them to Talon native unit
        //               based on wheel's effective diameter 5.6 ==> 3.14 * 5.6 * 2.54 = 45 cm / round,  optical sensor 1440 unit / round ---> 1 meter / (0.45m) x 1440 = 3200 unit / meter
      
        /* Insert every point into buffer, no limit on size */
        for (int i = 0; i < totalCnt; ++i) {
            String[] oneList = profile.get(i);
            double direction = forward ? +1 : -1;
            double leftPosition = Double.valueOf(oneList[1]) ; //m
            double leftVelocity = Double.valueOf(oneList[2]) ; //m/s
            double rightPosition = Double.valueOf(oneList[3]) ; //m
            double rightVelocity = Double.valueOf(oneList[4]) ;
            int durationMilliseconds = Integer.valueOf(oneList[7]);

            leftPoint.timeDur = durationMilliseconds;
            rightPoint.timeDur = durationMilliseconds;
       
            // Our MP's unit is Meter, Meter/second,   not Rount or Revolution, assuming 5.6 diameter wheel
            leftPoint.position = direction * leftPosition * RobotMap.kMeterToFalconSenorUnit; // Convert meter to native unit 
            leftPoint.velocity = direction * leftVelocity * RobotMap.kMeterToFalconSenorUnit / 10.0; // Convert Meter/second to native unit per 100 ms
            
            leftPoint.auxiliaryPos = 0;
            leftPoint.auxiliaryVel = 0;
            leftPoint.profileSlotSelect0 = RobotMap.kSlotIDx; /* which set of gains would you like to use [0,3]? */
            leftPoint.profileSlotSelect1 = 0; /* auxiliary PID [0,1], leave zero */
            leftPoint.zeroPos = (i == 0); /* set this to true on the first point */
            leftPoint.isLastPoint = ((i + 1) == totalCnt); /* set this to true on the last point */
            leftPoint.arbFeedFwd = 0; /* you can add a constant offset to add to PID[0] output here */

            _leftBufferedStream.Write(leftPoint);

            rightPoint.position = direction * rightPosition * RobotMap.kMeterToFalconSenorUnit; // Convert meter to native unit 
            rightPoint.velocity = direction * rightVelocity * RobotMap.kMeterToFalconSenorUnit / 10.0; // Convert Meter/second to native unit per 100 ms
            
            rightPoint.auxiliaryPos = 0;
            rightPoint.auxiliaryVel = 0;
            rightPoint.profileSlotSelect0 = RobotMap.kSlotIDx; /* which set of gains would you like to use [0,3]? */
            rightPoint.profileSlotSelect1 = 0; /* auxiliary PID [0,1], leave zero */
            rightPoint.zeroPos = (i == 0); /* set this to true on the first point */
            rightPoint.isLastPoint = ((i + 1) == totalCnt); /* set this to true on the last point */
            rightPoint.arbFeedFwd = 0; /* you can add a constant offset to add to PID[0] output here */

            _rightBufferedStream.Write(rightPoint);
        }
    }
}
