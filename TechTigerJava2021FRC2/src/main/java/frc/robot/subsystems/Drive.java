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
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
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


import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrame;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
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

  //public TalonSRX leftFrontTalon = new TalonSRX(RobotMap.leftTalonMaster);
  //public TalonSRX leftBackTalon = new TalonSRX(RobotMap.leftTalonSlave);
  //public TalonSRX rightFrontTalon = new TalonSRX(RobotMap.rightTalonMaster);
  //public TalonSRX rightBackTalon = new TalonSRX(RobotMap.rightTalonSlave);

  public WPI_TalonFX leftFrontTalon = new WPI_TalonFX(RobotMap.leftTalonMaster);
  public WPI_TalonFX leftBackTalon = new WPI_TalonFX(RobotMap.leftTalonSlave);
  public WPI_TalonFX rightFrontTalon = new WPI_TalonFX(RobotMap.rightTalonMaster);
  public WPI_TalonFX rightBackTalon = new WPI_TalonFX(RobotMap.rightTalonSlave);
  

  public TalonSRX vinnieTalon = new TalonSRX(RobotMap.vinnieTalonNumber);

  public PigeonIMU pigeonVinnie = new PigeonIMU(vinnieTalon);

  // ramsete related variables //
  DifferentialDrive m_drive ;
  //DifferentialDriveKinematics kinematics2 = new DifferentialDriveKinematics(Constants.DriveConstants.kTrackwidthMeters);
  //DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  //SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts, Constants.DriveConstants.kvVoltSecondsPerMeter, Constants.DriveConstants.kaVoltSecondsSquaredPerMeter);

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
 
  //double _pigeonRemoteSensoreScaleFactor = 1;
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
  //volatile int _state = 0;
  AtomicInteger _state = new AtomicInteger(0);

  // use Pneumatic PCM port 4, 5 to turn on/off LED light when the robot is within the low goal port distance
  //private Solenoid ledSolenoid = new Solenoid(4); // pinklight   ON/OFF
  //private AnalogInput analogLowGoalDistanceSensor; // measure the distance to low goal openning, if within range, turn on pink LED, otherwise, turn off
  public double [] yawPitchRollArrayStarting = null;

  
   @Override
    public void periodic() {

    }
  
   public Drive() {
      
      //readMPFile(false ); // false ==> not use Arc or pigeon
      readMPFile(RobotMap.kUseMotionProfileArc ); 


      pigeonVinnie.configFactoryDefault();
      pigeonVinnie.setYaw(0.0);
      pigeonVinnie.setFusedHeading(0.0);


     if( RobotMap.kUseMotionProfileArc == false) {
        configureDrive();
     }
     else {
      configureArcFXDrive();
     }

     
     m_drive = new DifferentialDrive(leftFrontTalon, rightFrontTalon);
     m_drive.setSafetyEnabled(false);
     leftFrontTalon.setSafetyEnabled(false);
     rightFrontTalon.setSafetyEnabled(false);

  }

  
  public void configureDrive() {
      leftFrontTalon.configFactoryDefault();
      leftBackTalon.configFactoryDefault();
      rightFrontTalon.configFactoryDefault();
      rightBackTalon.configFactoryDefault();
     
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
      leftFrontTalon.config_kP(0,0.049,30); //0.095
      leftFrontTalon.config_kI(0,0,30);
      leftFrontTalon.config_kD(0,0,30);

      rightFrontTalon.config_kF(0,0.045,30); // 0.045
      rightFrontTalon.config_kP(0,0.049,30); //0.095
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
    //if( analogLowGoalDistanceSensor != null) {
    //  cmDistanceSensor = analogLowGoalDistanceSensor.getAverageValue();
    //  SmartDashboard.putNumber("Analog Low Goal Distance Sensor  raw", cmDistanceSensor);  
    //}
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
    //return Rotation2d.fromDegrees(Math.IEEEremainder(ypr[0], 360.0d));

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
  
    // IF  motor config has only PID, no F (feedforward term), you can use the following way to mimic Feedforward term
    // OPTION #2: use WPIlib's SimpleMotorFeedforward to calculate the Feedforward term instead of a simple PIDF constant
    //  V = kS * sign(velocity) + kV * velocity + kA * acceleration      https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html
    // Ref: https://github.com/STMARobotics/frc-7028-2020/blob/master/src/main/java/frc/robot/subsystems/DriveTrainSubsystem.java 
    
    //var leftFeedForwardVolts = FEED_FORWARD.calculate(left);
    //var rightFeedForwardVolts = FEED_FORWARD.calculate(right);
    //leftFrontTalon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms_left, DemandType.ArbitraryFeedForward, leftFeedForwardVolts / 12);
    //rightFrontTalon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms_right, DemandType.ArbitraryFeedForward, rightFeedForwardVolts / 12);); 
  
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
        // do we need this ?
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

      // do we need this?
      leftFrontTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.pidLoopTimeout);
      rightFrontTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.pidLoopTimeout);

      if (pathNumber == 1){
        if( RobotMap.kUseMotionProfileArc == false) {
          leftFrontTalon.startMotionProfile(_leftBufferedStream1, 10, ControlMode.MotionProfile);
          rightFrontTalon.startMotionProfile(_rightBufferedStream1, 10, ControlMode.MotionProfile);
        }
        else {
          leftFrontTalon.startMotionProfile(_leftBufferedStream1, 10, TalonFXControlMode.MotionProfileArc.toControlMode());
          rightFrontTalon.startMotionProfile(_leftBufferedStream1, 10, TalonFXControlMode.MotionProfileArc.toControlMode());
        }
      } else if (pathNumber == 2){     
        if( RobotMap.kUseMotionProfileArc == false) {
          leftFrontTalon.startMotionProfile(_leftBufferedStream2, 10, ControlMode.MotionProfile);
          rightFrontTalon.startMotionProfile(_rightBufferedStream2, 10, ControlMode.MotionProfile);
        }
        else {
          leftFrontTalon.startMotionProfile(_leftBufferedStream2, 10, TalonFXControlMode.MotionProfileArc.toControlMode());
          rightFrontTalon.startMotionProfile(_leftBufferedStream2, 10, TalonFXControlMode.MotionProfileArc.toControlMode());
        }
      }
      else if (pathNumber == 3){
        if( RobotMap.kUseMotionProfileArc == false) {
          leftFrontTalon.startMotionProfile(_leftBufferedStream3, 10, ControlMode.MotionProfile);
          rightFrontTalon.startMotionProfile(_rightBufferedStream3, 10, ControlMode.MotionProfile);
        }
        else {
          leftFrontTalon.startMotionProfile(_leftBufferedStream3, 10, TalonFXControlMode.MotionProfileArc.toControlMode());
          rightFrontTalon.startMotionProfile(_leftBufferedStream3, 10, TalonFXControlMode.MotionProfileArc.toControlMode());
        }
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
      /*Convert the initial (x,y) coordinates to polar coordinates.
        Rotate them by 45 degrees.
        Convert the polar coordinates back to cartesian.
        Rescale the new coordinates to -1.0/+1.0.
        Clamp the new values to -1.0/+1.0.
        This assumes the initial (x,y) coordinates are in the -1.0/+1.0 range. The side of the inner square will always be 
        equal to l * sqrt(2)/2, so step 4 is just about multiplying the values by sqrt(2). */
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
              if(useArc == false) {
                initBuffer(arrayList1, totalCnt, false, 0, _leftBufferedStream1, _rightBufferedStream1);
              }
              else {
                initBufferArc(arrayList1, totalCnt, false, 0, _leftBufferedStream1, _rightBufferedStream1);
              }
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
                if(useArc == false) {
                  initBuffer(arrayList2, totalCnt, true, 1, _leftBufferedStream2, _rightBufferedStream2);
                }
                else {
                  initBufferArc(arrayList2, totalCnt, true, 1, _leftBufferedStream2, _rightBufferedStream2);
                }
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
                if(useArc == false) {
                   initBuffer(arrayList3, totalCnt, false, 1, _leftBufferedStream3, _rightBufferedStream3);
                }
                else {
                  initBufferArc(arrayList3, totalCnt, false, 1, _leftBufferedStream3, _rightBufferedStream3);
                }
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
                if(useArc == false) {
                  initBuffer(arrayList4, totalCnt, true, 1, _leftBufferedStream4, _rightBufferedStream4);
                }
                else {
                  initBufferArc(arrayList4, totalCnt, true, 1, _leftBufferedStream4, _rightBufferedStream4);
                }
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


    private void initBufferArc(ArrayList<String[]> profile, int totalCnt, boolean forward, int slotNumber, BufferedTrajectoryPointStream _leftBufferedStream, BufferedTrajectoryPointStream _rightBufferedStream) {
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
           double leftAngle = Double.valueOf(oneList[5]) ;
           double rightAngle = Double.valueOf(oneList[6]) ;            
           int durationMilliseconds = Integer.valueOf(oneList[7]);

           leftPoint.timeDur = durationMilliseconds;
           rightPoint.timeDur = durationMilliseconds;
      
           // Our MP's unit is Meter, Meter/second,   not Rount or Revolution, assuming 5.6 diameter wheel
           leftPoint.position = direction * leftPosition * RobotMap.kMeterToFalconSenorUnit; // Convert meter to native unit 
           leftPoint.velocity = direction * leftVelocity * RobotMap.kMeterToFalconSenorUnit / 10.0; // Convert Meter/second to native unit per 100 ms
           leftPoint.arbFeedFwd = 0; /* you can add a constant offset to add to PID[0] output here */

           leftPoint.auxiliaryPos =  leftAngle  * 22.755; //Constants.kTurnUnitsPerDeg;
           leftPoint.auxiliaryVel = 0;
           //leftPoint.profileSlotSelect0 = RobotMap.kSlotIDx; /* which set of gains would you like to use [0,3]? */
           //leftPoint.profileSlotSelect1 = 0; /* auxiliary PID [0,1], leave zero */
           //leftPoint.zeroPos = (i == 0); /* set this to true on the first point */
           //leftPoint.isLastPoint = ((i + 1) == totalCnt); /* set this to true on the last point */
          
           leftPoint.auxiliaryArbFeedFwd = 0;
           leftPoint.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
           leftPoint.profileSlotSelect1 = 1; /* auxiliary PID [0,1], leave zero */
           leftPoint.zeroPos = false; /* don't reset sensor, this is done elsewhere since we have multiple sensors */
           leftPoint.isLastPoint = ((i + 1) == totalCnt); /* set this to true on the last point */
           leftPoint.useAuxPID = true; /* tell MPB that we are using both pids */


           _leftBufferedStream.Write(leftPoint);


           rightPoint.position = direction * rightPosition * RobotMap.kMeterToFalconSenorUnit; // Convert meter to native unit 
           rightPoint.velocity = direction * rightVelocity * RobotMap.kMeterToFalconSenorUnit / 10.0; // Convert Meter/second to native unit per 100 ms
           rightPoint.arbFeedFwd = 0; /* you can add a constant offset to add to PID[0] output here */

           rightPoint.auxiliaryPos = rightAngle * 22.755;// Constants.kTurnUnitsPerDeg;
           rightPoint.auxiliaryVel = 0;
           rightPoint.auxiliaryArbFeedFwd = 0;
           
           rightPoint.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
           rightPoint.profileSlotSelect1 = 1; /* auxiliary PID [0,1], leave zero */
           rightPoint.zeroPos = false; /* don't reset sensor, this is done elsewhere since we have multiple sensors */
           rightPoint.isLastPoint = ((i + 1) == totalCnt); /* set this to true on the last point */
           rightPoint.useAuxPID = true; /* tell MPB that we are using both pids */
           
           _rightBufferedStream.Write(rightPoint);
       }
   }



   public void configureArcFXDrive() {
    configureArcFXDrive(true);
   }
   public void configureArcFXDrive(boolean resetSensor) {
    leftFrontTalon.configFactoryDefault();
    leftBackTalon.configFactoryDefault();
    rightFrontTalon.configFactoryDefault();
    rightBackTalon.configFactoryDefault();
   
    
    leftFrontTalon.set(ControlMode.PercentOutput, 0);
    rightFrontTalon.set(ControlMode.PercentOutput, 0);
    
    leftFrontTalon.configNeutralDeadband(0.05,30);
    rightFrontTalon.configNeutralDeadband(0.05,30);
    leftBackTalon.configNeutralDeadband(0.0,30);
    rightBackTalon.configNeutralDeadband(0.0,30);
    
    leftBackTalon.follow(leftFrontTalon);
    rightBackTalon.follow(rightFrontTalon);

    leftFrontTalon.configClosedloopRamp(RobotMap.talonDriveAccelerationRate );
    rightFrontTalon.configClosedloopRamp(RobotMap.talonDriveAccelerationRate );
    rightFrontTalon.configOpenloopRamp(RobotMap.talonDriveAccelerationRate );
    leftFrontTalon.configOpenloopRamp(RobotMap.talonDriveAccelerationRate );
    
    rightFrontTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, RobotMap.pidLoopTimeout);//0 is the primary PID index
    leftFrontTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, RobotMap.pidLoopTimeout);

    leftFrontTalon.selectProfileSlot(0,0);
    rightFrontTalon.selectProfileSlot(0,0);

    // copied from Arc setup



    
    //  sensor:  integrated and remote
 
    //leftFrontTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,  RobotMap.PID_PRIMARY, RobotMap.pidLoopTimeout);
    leftFrontTalon.configSelectedFeedbackCoefficient(1, 	RobotMap.PID_PRIMARY,	RobotMap.pidLoopTimeout);

    leftFrontTalon.configRemoteFeedbackFilter(0,	RemoteSensorSource.Off, RobotMap.REMOTE_0,	RobotMap.pidLoopTimeout);

    leftFrontTalon.configRemoteFeedbackFilter(pigeonVinnie.getDeviceID(),	RemoteSensorSource.GadgeteerPigeon_Yaw, RobotMap.REMOTE_1,	RobotMap.pidLoopTimeout);

    leftFrontTalon.configSelectedFeedbackSensor(	FeedbackDevice.RemoteSensor1, RobotMap.PID_TURN,  RobotMap.pidLoopTimeout);
    // Coefficient has HUGE impact on the PID:   difference x Coefficient,  3600/8192,  so if coefficient is 1, it will be 22.7 sensor unit / degree ==> more aggressive PID correction, if x (3600/8194), ==> sensor unit is 10 unit/degree
    leftFrontTalon.configSelectedFeedbackCoefficient(	_pigeonRemoteSensoreScaleFactor, RobotMap.PID_TURN, RobotMap.pidLoopTimeout);



    rightFrontTalon.configRemoteFeedbackFilter(leftFrontTalon.getDeviceID(),	RemoteSensorSource.TalonFX_SelectedSensor,	RobotMap.REMOTE_0,	RobotMap.kTimeoutMs);	
    rightFrontTalon.configRemoteFeedbackFilter(pigeonVinnie.getDeviceID(),RemoteSensorSource.GadgeteerPigeon_Yaw,RobotMap.REMOTE_1,RobotMap.kTimeoutMs);
    rightFrontTalon.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, RobotMap.pidLoopTimeout);			
    rightFrontTalon.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.IntegratedSensor, RobotMap.pidLoopTimeout);

    rightFrontTalon.configSelectedFeedbackSensor(	FeedbackDevice.IntegratedSensor, RobotMap.PID_PRIMARY, RobotMap.pidLoopTimeout);
    rightFrontTalon.configSelectedFeedbackCoefficient(1, 	RobotMap.PID_PRIMARY,	RobotMap.pidLoopTimeout);

    rightFrontTalon.configSelectedFeedbackSensor(	FeedbackDevice.RemoteSensor1,  RobotMap.PID_TURN,   RobotMap.pidLoopTimeout);
    rightFrontTalon.configSelectedFeedbackCoefficient(	_pigeonRemoteSensoreScaleFactor, RobotMap.PID_TURN, RobotMap.pidLoopTimeout);


    //drive     PIDF  kSlotIDx = 0
    leftFrontTalon.config_kF(RobotMap.kSlotIDx, RobotMap.driveGainsVelocity.kF, RobotMap.pidLoopTimeout);	    
    leftFrontTalon.config_kP(RobotMap.kSlotIDx, RobotMap.driveGainsVelocity.kP, RobotMap.pidLoopTimeout);	    
    leftFrontTalon.config_kI(RobotMap.kSlotIDx, RobotMap.driveGainsVelocity.kI, RobotMap.pidLoopTimeout);	    
    leftFrontTalon.config_kD(RobotMap.kSlotIDx, RobotMap.driveGainsVelocity.kD, RobotMap.pidLoopTimeout);	    



    leftFrontTalon.config_IntegralZone(RobotMap.kSlotIDx, RobotMap.driveGainsVelocity.kIzone, RobotMap.pidLoopTimeout);
    leftFrontTalon.configClosedLoopPeakOutput(RobotMap.kSlotIDx, RobotMap.driveGainsVelocity.kPeakOutput, RobotMap.pidLoopTimeout);
    leftFrontTalon.configAllowableClosedloopError(RobotMap.kSlotIDx, 0, RobotMap.pidLoopTimeout);

    rightFrontTalon.config_kF(RobotMap.kSlotIDx, RobotMap.driveGainsVelocity.kF, RobotMap.pidLoopTimeout);	   
    rightFrontTalon.config_kP(RobotMap.kSlotIDx, RobotMap.driveGainsVelocity.kP, RobotMap.pidLoopTimeout);	    
    rightFrontTalon.config_kI(RobotMap.kSlotIDx, RobotMap.driveGainsVelocity.kI, RobotMap.pidLoopTimeout);	    
    rightFrontTalon.config_kD(RobotMap.kSlotIDx, RobotMap.driveGainsVelocity.kD, RobotMap.pidLoopTimeout);	   


    rightFrontTalon.config_IntegralZone(RobotMap.kSlotIDx, RobotMap.driveGainsVelocity.kIzone, RobotMap.pidLoopTimeout);
    rightFrontTalon.configClosedLoopPeakOutput(RobotMap.kSlotIDx, RobotMap.driveGainsVelocity.kPeakOutput, RobotMap.pidLoopTimeout);
    rightFrontTalon.configAllowableClosedloopError(RobotMap.kSlotIDx, 0, RobotMap.pidLoopTimeout);

    //turning   PIDF  kTurnAutonomousSlotIDx = 1
    leftFrontTalon.config_kF(RobotMap.kTurnAutonomousSlotIDx, RobotMap.turnGainsVelocity.kF);
    leftFrontTalon.config_kP(RobotMap.kTurnAutonomousSlotIDx, RobotMap.turnGainsVelocity.kP);
    leftFrontTalon.config_kI(RobotMap.kTurnAutonomousSlotIDx, RobotMap.turnGainsVelocity.kI);
    leftFrontTalon.config_kD(RobotMap.kTurnAutonomousSlotIDx, RobotMap.turnGainsVelocity.kD);

    leftFrontTalon.config_IntegralZone(RobotMap.kTurnAutonomousSlotIDx, RobotMap.turnGainsVelocity.kIzone, RobotMap.pidLoopTimeout);
    leftFrontTalon.configClosedLoopPeakOutput(RobotMap.kTurnAutonomousSlotIDx, RobotMap.turnGainsVelocity.kPeakOutput, RobotMap.pidLoopTimeout);
    leftFrontTalon.configAllowableClosedloopError(RobotMap.kTurnAutonomousSlotIDx, 0, RobotMap.pidLoopTimeout);


    rightFrontTalon.config_kF(RobotMap.kTurnAutonomousSlotIDx, RobotMap.turnGainsVelocity.kF);
    rightFrontTalon.config_kP(RobotMap.kTurnAutonomousSlotIDx, RobotMap.turnGainsVelocity.kP);
    rightFrontTalon.config_kI(RobotMap.kTurnAutonomousSlotIDx, RobotMap.turnGainsVelocity.kI);
    rightFrontTalon.config_kD(RobotMap.kTurnAutonomousSlotIDx, RobotMap.turnGainsVelocity.kD);

    rightFrontTalon.config_IntegralZone(RobotMap.kTurnAutonomousSlotIDx, RobotMap.turnGainsVelocity.kIzone, RobotMap.pidLoopTimeout);
    rightFrontTalon.configClosedLoopPeakOutput(RobotMap.kTurnAutonomousSlotIDx, RobotMap.turnGainsVelocity.kPeakOutput, RobotMap.pidLoopTimeout);
    rightFrontTalon.configAllowableClosedloopError(RobotMap.kTurnAutonomousSlotIDx, 0, RobotMap.pidLoopTimeout);


    // frame status  speed up the target polling for PID[0] and PID-aux[1]

    pigeonVinnie.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 5, RobotMap.pidLoopTimeout);


    rightFrontTalon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, RobotMap.pidLoopTimeout);
		rightFrontTalon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, RobotMap.pidLoopTimeout);
		rightFrontTalon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, RobotMap.pidLoopTimeout);
		// Status_10_Targets is for motion magic only
		rightFrontTalon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, RobotMap.pidLoopTimeout);
    rightFrontTalon.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10);
    rightFrontTalon.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 20);//?


     leftFrontTalon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, RobotMap.pidLoopTimeout);
     leftFrontTalon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, RobotMap.pidLoopTimeout);
     leftFrontTalon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, RobotMap.pidLoopTimeout);
		// Status_10_Targets is for motion magic only
		leftFrontTalon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, RobotMap.pidLoopTimeout);
    leftFrontTalon.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10);
    leftFrontTalon.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 20);//?

    
    leftFrontTalon.setSelectedSensorPosition(0, RobotMap.kSlotIDx, RobotMap.pidLoopTimeout);
		rightFrontTalon.setSelectedSensorPosition(0, RobotMap.kSlotIDx, RobotMap.pidLoopTimeout);

    int closedLoopTimeMs = 1;
		rightFrontTalon.configClosedLoopPeriod(RobotMap.kSlotIDx, closedLoopTimeMs, RobotMap.pidLoopTimeout);
		rightFrontTalon.configClosedLoopPeriod(RobotMap.kTurnAutonomousSlotIDx, closedLoopTimeMs, RobotMap.pidLoopTimeout);

		leftFrontTalon.configClosedLoopPeriod(RobotMap.kSlotIDx, closedLoopTimeMs, RobotMap.pidLoopTimeout);
		leftFrontTalon.configClosedLoopPeriod(RobotMap.kTurnAutonomousSlotIDx, closedLoopTimeMs, RobotMap.pidLoopTimeout);


   // end of Arc setup copy




    //leftFrontTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
    //leftFrontTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);

    //rightFrontTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
    //rightFrontTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);




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
  


    leftFrontTalon.configMotionCruiseVelocity(4000, 30); // decreased from 8000 which is about 1.7 m/s
    leftFrontTalon.configMotionAcceleration(4000, 30);

    rightFrontTalon.configMotionCruiseVelocity(4000, 30);
    rightFrontTalon.configMotionAcceleration(4000, 30);


   
    // copy from Arc setup for configAuxPIDPolarity and Profile Slot  ????
    rightFrontTalon.configAuxPIDPolarity(false, RobotMap.pidLoopTimeout);
	 	leftFrontTalon.configAuxPIDPolarity(true, RobotMap.pidLoopTimeout); // left is independent, need flip the polarityb??  not sure for FX, sample code has not polarity setting


   // rightFrontTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);
   // leftFrontTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);


    // the following two are from regular one
    leftFrontTalon.selectProfileSlot(RobotMap.kSlotIDx, RobotMap.PID_PRIMARY); // not kPIDLoopIDx (=0)
    rightFrontTalon.selectProfileSlot(RobotMap.kSlotIDx, RobotMap.PID_PRIMARY);

    // the following are new for Arc setup
    leftFrontTalon.selectProfileSlot(RobotMap.kTurnAutonomousSlotIDx, RobotMap.PID_TURN);
    rightFrontTalon.selectProfileSlot(RobotMap.kTurnAutonomousSlotIDx, RobotMap.PID_TURN);

    if (resetSensor) {
      zeroSensors();
    }
 }


}
