/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.Robot;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motorcontrol.InvertType; 
import com.ctre.phoenix.motorcontrol.ControlMode; 
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

//import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
//import com.ctre.phoenix.motorcontrol.SensorTerm;
//import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
//import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import java.io.*;
import java.util.ArrayList;
//import com.ctre.phoenix.motorcontrol.*;
//import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motion.*;
import java.util.concurrent.atomic.AtomicInteger;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Add your docs here.
 */
public class Drive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands. heehee

  public DifferentialDrive differentialDrive;
  public double leftSpeed; 
  public double rightSpeed;

  //public TalonSRX leftFrontTalon = new TalonSRX(RobotMap.leftTalonMaster);
  //public TalonSRX leftBackTalon = new TalonSRX(RobotMap.leftTalonSlave);
  //public TalonSRX rightFrontTalon = new TalonSRX(RobotMap.rightTalonMaster);
  //public TalonSRX rightBackTalon = new TalonSRX(RobotMap.rightTalonSlave);

  public TalonFX leftFrontTalon = new TalonFX(RobotMap.leftTalonMaster);
  public TalonFX leftBackTalon = new TalonFX(RobotMap.leftTalonSlave);
  public TalonFX rightFrontTalon = new TalonFX(RobotMap.rightTalonMaster);
  public TalonFX rightBackTalon = new TalonFX(RobotMap.rightTalonSlave);
  

  public TalonSRX vinnieTalon = new TalonSRX(RobotMap.vinnieTalonNumber);

  public PigeonIMU pigeonVinnie = new PigeonIMU(vinnieTalon);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  public double Kp = -0.1;
  public double min_command = 0.05;
 
  double _pigeonRemoteSensoreScaleFactor = 1;

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
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ManualDriveCommand());
   }

   public Drive() {
    
      readMPFile();

      pigeonVinnie.configFactoryDefault();
      pigeonVinnie.setYaw(0.0);
      pigeonVinnie.setFusedHeading(0.0);
      configureDrive();

      //analogLowGoalDistanceSensor = new AnalogInput(1);//RobotMap.analogLowGoalDistanceSensorPort
      //analogLowGoalDistanceSensor.setAverageBits(20);

    /*
        readMPFile();

        leftBackTalon.configFactoryDefault();
        rightBackTalon.configFactoryDefault();
        
        leftBackTalon.configNeutralDeadband(0.0,30);
        rightBackTalon.configNeutralDeadband(0.0,30);
        
        leftBackTalon.follow(leftFrontTalon);
        rightBackTalon.follow(rightFrontTalon);
        
        leftBackTalon.setSensorPhase(false);
        rightBackTalon.setSensorPhase(true);
        
        leftBackTalon.setInverted(InvertType.FollowMaster);
        rightBackTalon.setInverted(InvertType.FollowMaster);


        _leftMasterconfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        _leftMasterconfig.neutralDeadband = RobotMap.kNeutralDeadband; 
        _leftMasterconfig.slot0.kF = RobotMap.kGains_MotProf.kF;
        _leftMasterconfig.slot0.kP = RobotMap.kGains_MotProf.kP;
        _leftMasterconfig.slot0.kI = RobotMap.kGains_MotProf.kI;
        _leftMasterconfig.slot0.kD = RobotMap.kGains_MotProf.kD;
        _leftMasterconfig.slot0.integralZone = (int) RobotMap.kGains_MotProf.kIzone;
        _leftMasterconfig.slot0.closedLoopPeakOutput = RobotMap.kGains_MotProf.kPeakOutput; 
        // _config.slot0.allowableClosedloopError // left default for this example
        // _config.slot0.maxIntegralAccumulator; // left default for this example
        // _config.slot0.closedLoopPeriod; // left default for this example
        leftFrontTalon.configAllSettings(_leftMasterconfig);
     
        //_master.setSensorPhase(true);
        leftFrontTalon.setSensorPhase(false);
        leftFrontTalon.setInverted(false);

        leftFrontTalon.configClosedloopRamp(1);
        rightFrontTalon.configClosedloopRamp(1);
        rightFrontTalon.configOpenloopRamp(1);
        leftFrontTalon.configOpenloopRamp(1);

        leftFrontTalon.configMotionCruiseVelocity(8000, 30);
        leftFrontTalon.configMotionAcceleration(8000, 30);
    
        rightFrontTalon.configMotionCruiseVelocity(8000, 30);
        rightFrontTalon.configMotionAcceleration(8000, 30);

        _rightMasterconfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        _rightMasterconfig.neutralDeadband = RobotMap.kNeutralDeadband; 
        _rightMasterconfig.slot0.kF = RobotMap.kGains_MotProf.kF;
        _rightMasterconfig.slot0.kP = RobotMap.kGains_MotProf.kP;
        _rightMasterconfig.slot0.kI = RobotMap.kGains_MotProf.kI;
        _rightMasterconfig.slot0.kD = RobotMap.kGains_MotProf.kD;
        _rightMasterconfig.slot0.integralZone = (int) RobotMap.kGains_MotProf.kIzone;
        _rightMasterconfig.slot0.closedLoopPeakOutput = RobotMap.kGains_MotProf.kPeakOutput;

        rightFrontTalon.configAllSettings(_rightMasterconfig);

        rightFrontTalon.setInverted(true);

        leftFrontTalon.selectProfileSlot(RobotMap.kSlotIDx, RobotMap.kPIDLoopIDx);
        rightFrontTalon.selectProfileSlot(RobotMap.kSlotIDx, RobotMap.kPIDLoopIDx);

        zeroSensors();

        try {
            Thread.sleep(1000);// sleep for one second to let the MP buffer to finish
        }
        catch (Exception e) {

        }
*/


   /*
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

		leftFrontTalon.config_kF(0,0.0455,30);
		leftFrontTalon.config_kP(0,0.095,30);
		leftFrontTalon.config_kI(0,0,30);
		leftFrontTalon.config_kD(0,0,30);

		rightFrontTalon.config_kF(0,0.0455,30);
		rightFrontTalon.config_kP(0,0.095,30);
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
    */
    

/*
    leftFrontTalon.configFactoryDefault();
    leftBackTalon.configFactoryDefault();
    rightFrontTalon.configFactoryDefault();
    rightBackTalon.configFactoryDefault();
    pigeonVinnie.configFactoryDefault();
    pigeonVinnie.setYaw(0.0);
    pigeonVinnie.setFusedHeading(0.0);
    pigeonVinnie.setAccumZAngle(0, RobotMap.pidLoopTimeout);
    
    leftFrontTalon.set(ControlMode.PercentOutput, 0);
		rightFrontTalon.set(ControlMode.PercentOutput, 0);
		
    
    leftFrontTalon.configNeutralDeadband(RobotMap.driveDeadband, RobotMap.pidLoopTimeout);
		rightFrontTalon.configNeutralDeadband(RobotMap.driveDeadband, RobotMap.pidLoopTimeout);
    //DON'T SET FOLLOWERS DEADBAND TO NON-ZERO
    leftBackTalon.configNeutralDeadband(0, RobotMap.pidLoopTimeout);
    rightBackTalon.configNeutralDeadband(0, RobotMap.pidLoopTimeout);
    

    
    leftBackTalon.follow(leftFrontTalon);
    rightBackTalon.follow(rightFrontTalon);


    leftFrontTalon.configClosedloopRamp(RobotMap.talonDriveAccelerationRate);
		rightFrontTalon.configClosedloopRamp(RobotMap.talonDriveAccelerationRate);
		rightFrontTalon.configOpenloopRamp(RobotMap.talonDriveAccelerationRate);
    leftFrontTalon.configOpenloopRamp(RobotMap.talonDriveAccelerationRate);

    
    //  sensor:  integrated and remote
 
    leftFrontTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,  RobotMap.PID_PRIMARY, RobotMap.pidLoopTimeout);
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

    rightFrontTalon.configSelectedFeedbackSensor(	FeedbackDevice.RemoteSensor1,RobotMap.PID_TURN,RobotMap.pidLoopTimeout);
    rightFrontTalon.configSelectedFeedbackCoefficient(	_pigeonRemoteSensoreScaleFactor, RobotMap.PID_TURN, RobotMap.pidLoopTimeout);


    leftFrontTalon.setInverted(false);
    rightFrontTalon.setInverted(true);

    leftBackTalon.setInverted(InvertType.FollowMaster);
    rightBackTalon.setInverted(InvertType.FollowMaster);

    rightFrontTalon.setSensorPhase(false);
    leftFrontTalon.setSensorPhase(false);
    rightBackTalon.setSensorPhase(false);
    leftBackTalon.setSensorPhase(false);


    // frame status

    pigeonVinnie.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 5, RobotMap.pidLoopTimeout);


    rightFrontTalon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, RobotMap.pidLoopTimeout);
		rightFrontTalon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, RobotMap.pidLoopTimeout);
		rightFrontTalon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, RobotMap.pidLoopTimeout);
		// Status_10_Targets is for motion magic only
		rightFrontTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.pidLoopTimeout);
		rightFrontTalon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, RobotMap.pidLoopTimeout);

     leftFrontTalon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, RobotMap.pidLoopTimeout);
     leftFrontTalon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, RobotMap.pidLoopTimeout);
     leftFrontTalon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, RobotMap.pidLoopTimeout);
		// Status_10_Targets is for motion magic only
		leftFrontTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.pidLoopTimeout);
		leftFrontTalon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, RobotMap.pidLoopTimeout);


    
    leftFrontTalon.configMotionCruiseVelocity(8000, RobotMap.pidLoopTimeout);
		leftFrontTalon.configMotionAcceleration(8000, RobotMap.pidLoopTimeout);

		rightFrontTalon.configMotionCruiseVelocity(8000, RobotMap.pidLoopTimeout);
    rightFrontTalon.configMotionAcceleration(8000, RobotMap.pidLoopTimeout);

    

    //drive     PIDF 
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

    //turning   PIDF
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



    int closedLoopTimeMs = 1;
    rightFrontTalon.configClosedLoopPeriod(RobotMap.kSlotIDx, closedLoopTimeMs, RobotMap.pidLoopTimeout);
		rightFrontTalon.configClosedLoopPeriod(RobotMap.kTurnAutonomousSlotIDx, closedLoopTimeMs,RobotMap.pidLoopTimeout);

		leftFrontTalon.configClosedLoopPeriod(RobotMap.kSlotIDx, closedLoopTimeMs, RobotMap.pidLoopTimeout);
		leftFrontTalon.configClosedLoopPeriod(RobotMap.kTurnAutonomousSlotIDx, closedLoopTimeMs, RobotMap.pidLoopTimeout);
  



    // Others



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


    //rightFrontTalon.configAuxPIDPolarity(false, RobotMap.pidLoopTimeout);
		//leftFrontTalon.configAuxPIDPolarity(false, RobotMap.pidLoopTimeout); // left is independent, need flip the polarity

		rightFrontTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);
    leftFrontTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);
    
  
    rightFrontTalon.selectProfileSlot(RobotMap.kSlotIDx, RobotMap.PID_PRIMARY);
		rightFrontTalon.selectProfileSlot(RobotMap.kTurnAutonomousSlotIDx, RobotMap.PID_TURN);

		leftFrontTalon.selectProfileSlot(RobotMap.kSlotIDx, RobotMap.PID_PRIMARY);
		leftFrontTalon.selectProfileSlot(RobotMap.kTurnAutonomousSlotIDx, RobotMap.PID_TURN);

    zeroSensors();
	 */

  }

  
  public void configureDrive() {
      leftFrontTalon.configFactoryDefault();
      leftBackTalon.configFactoryDefault();
      rightFrontTalon.configFactoryDefault();
      rightBackTalon.configFactoryDefault();
      //pigeonVinnie.configFactoryDefault();
      //pigeonVinnie.setYaw(0.0);
      //pigeonVinnie.setFusedHeading(0.0);
      
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

      leftFrontTalon.config_kF(0,0.0455,30);
      leftFrontTalon.config_kP(0,0.095,30);
      leftFrontTalon.config_kI(0,0,30);
      leftFrontTalon.config_kD(0,0,30);

      rightFrontTalon.config_kF(0,0.0455,30);
      rightFrontTalon.config_kP(0,0.095,30);
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
    // boolean angleIsGood = (_pidgey.getState() == PigeonIMU.PigeonState.Ready) ? true : false;
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
  
  public void setArcade(double velocity, double turn) {
      mercyArcadeDrive(velocity, turn);
    // differentialDrive.arcadeDrive(velocity, turn);
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
      rightFrontTalon.set(ControlMode.MotionMagic, distance, DemandType.AuxPID, turn_angle);		
      leftFrontTalon.set(ControlMode.MotionMagic, distance, DemandType.AuxPID, turn_angle);
    }
  }

  public boolean isMotionMagicDone(double targetDistanceInNativeUnit, boolean resetToPercentMode) {
    boolean ret = false;
    double sensorDistance = (leftFrontTalon.getSelectedSensorPosition(0) + rightFrontTalon.getSelectedSensorPosition(0))/2.0;
    double percentError = 100 * (targetDistanceInNativeUnit - sensorDistance)/targetDistanceInNativeUnit;

    // even though it is desired to achieve error < 1%, it depends on PID tuning, sometimes it is always achieable
    if (percentError < 2.5){
      if( resetToPercentMode == true) {
        // do we need this ?
        leftFrontTalon.getSensorCollection().setIntegratedSensorPosition(0,  RobotMap.pidLoopTimeout);
        rightFrontTalon.getSensorCollection().setIntegratedSensorPosition(0,  RobotMap.pidLoopTimeout);

        leftFrontTalon.set(ControlMode.PercentOutput, 0);
        rightFrontTalon.set(ControlMode.PercentOutput, 0);
      }
      return true;
    }
    return ret;
  }

  public void setMotionProfile(int pathNumber) {
  
    if( _state.get() != 2) {
      _state.set(2);

      // do we need this?
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
    String path1FileName = "";
    String path2FileName = "";
    String path3FileName = "";
    String path4FileName = "";
    //String autonomous = SmartDashboard.getString("Autonomous", "Default");
    int autonomous = (int) Math.round(SmartDashboard.getNumber("Autonomous", 1.0)); 
    //autonomous = "LeftTurn";
    //if (autonomous.equals("DriveStraight") || autonomous.equals("Default")){
    if (autonomous == 0 ) {  //DriveStraight      
      path1FileName = "/home/lvuser/mp_20ms_in_meter_arc_2_1.csv";
      path2FileName = "/home/lvuser/mp_20ms_in_meter_arc_2_2.csv";
    } else if (autonomous == 1){ //CenterDeploy
       path1FileName = "/home/lvuser/mp_20ms_in_meter_arc_1_1.csv";
       path2FileName = "/home/lvuser/mp_20ms_in_meter_arc_1_2.csv";
    } else if (autonomous == 2){ //RightDeploy
      path1FileName = "/home/lvuser/mp_20ms_in_meter_arc_3_1.csv";
      path2FileName = "/home/lvuser/mp_20ms_in_meter_arc_3_2.csv";
    } else if (autonomous == 3){ //LeftDeploy
      path1FileName = "/home/lvuser/mp_20ms_in_meter_arc_3_1.csv";
      //path2FileName = "/home/lvuser/mp_20ms_in_meter_arc_3_2.csv";
    } else if (autonomous == 4){ //LeftTurn
      path1FileName = "/home/lvuser/mp_20ms_in_meter_arc_4_1.csv";
      path2FileName = "/home/lvuser/mp_20ms_in_meter_arc_4_2.csv";
      path3FileName = "/home/lvuser/mp_20ms_in_meter_arc_4_3.csv";
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
