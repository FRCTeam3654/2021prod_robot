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
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
//import edu.wpi.first.wpilibj.Servo;

public class Climb extends SubsystemBase {
  private TalonSRX climbRightTalon = new TalonSRX(RobotMap.climbRightTalonID);
  private TalonSRX climbLeftTalon = new TalonSRX(RobotMap.climbLeftTalonID);

  public double leftSpeed; 
  public double rightSpeed;

  @Override
  public void periodic() {
   
   
  }

  public void setPower(double elevatorPower){
     }

  public Climb (){
    climbLeftTalon.configFactoryDefault();
    climbRightTalon.configFactoryDefault();

    climbLeftTalon.setNeutralMode(NeutralMode.Brake);
    climbRightTalon.setNeutralMode(NeutralMode.Brake);

    climbLeftTalon.setInverted(false);
    climbRightTalon.setInverted(false);

    climbLeftTalon.setSensorPhase(false);
    climbRightTalon.setSensorPhase(false);

    climbLeftTalon.setNeutralMode(NeutralMode.Brake);
    climbRightTalon.setNeutralMode(NeutralMode.Brake);

    climbLeftTalon.configNeutralDeadband(0.001, RobotMap.kTimeoutMs);
    climbRightTalon.configNeutralDeadband(0.001, RobotMap.kTimeoutMs);

    climbLeftTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, RobotMap.pidLoopTimeout);
    climbRightTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, RobotMap.pidLoopTimeout);

    climbLeftTalon.config_kF(RobotMap.kClimbSlotIDx,RobotMap.climbGainsVelocity.kF);
    climbLeftTalon.config_kP(RobotMap.kClimbSlotIDx,RobotMap.climbGainsVelocity.kP);
    climbLeftTalon.config_kI(RobotMap.kClimbSlotIDx,RobotMap.climbGainsVelocity.kI);
    climbLeftTalon.config_kD(RobotMap.kClimbSlotIDx,RobotMap.climbGainsVelocity.kD);

    climbRightTalon.config_kF(RobotMap.kClimbSlotIDx,RobotMap.climbGainsVelocity.kF);
    climbRightTalon.config_kP(RobotMap.kClimbSlotIDx,RobotMap.climbGainsVelocity.kP);
    climbRightTalon.config_kI(RobotMap.kClimbSlotIDx,RobotMap.climbGainsVelocity.kI);
    climbRightTalon.config_kD(RobotMap.kClimbSlotIDx,RobotMap.climbGainsVelocity.kD);

    climbLeftTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.kTimeoutMs);
    climbLeftTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.kTimeoutMs);
    climbRightTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.kTimeoutMs);
    climbRightTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.kTimeoutMs);

    /* set the peak and nominal outputs */
    climbLeftTalon.configNominalOutputForward(0, RobotMap.kTimeoutMs);
    climbLeftTalon.configNominalOutputReverse(0, RobotMap.kTimeoutMs);
    climbLeftTalon.configPeakOutputForward(1, RobotMap.kTimeoutMs);
    climbLeftTalon.configPeakOutputReverse(-1, RobotMap.kTimeoutMs);

    climbRightTalon.configNominalOutputForward(0, RobotMap.kTimeoutMs);
    climbRightTalon.configNominalOutputReverse(0, RobotMap.kTimeoutMs);
    climbRightTalon.configPeakOutputForward(1, RobotMap.kTimeoutMs);
    climbRightTalon.configPeakOutputReverse(-1, RobotMap.kTimeoutMs);

    climbLeftTalon.configMotionCruiseVelocity(RobotMap.climbCruiseVelocity, RobotMap.kTimeoutMs);
    climbLeftTalon.configMotionAcceleration(RobotMap.climbAcceleration, RobotMap.kTimeoutMs);
    climbRightTalon.configMotionCruiseVelocity(RobotMap.climbCruiseVelocity, RobotMap.kTimeoutMs);
    climbRightTalon.configMotionAcceleration(RobotMap.climbAcceleration, RobotMap.kTimeoutMs);

    climbLeftTalon.selectProfileSlot(RobotMap.kClimbSlotIDx, RobotMap.kPIDLoopIDx);
    climbRightTalon.selectProfileSlot(RobotMap.kClimbSlotIDx, RobotMap.kPIDLoopIDx);
    /* zero the sensor once on robot boot up*/
    climbLeftTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.kTimeoutMs);
    climbLeftTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.kTimeoutMs);
    
    zeroSensors();
  }

  public void setArcade(double velocity, double turn){
    mercyArcadeDrive(velocity, turn);
   // differentialDrive.arcadeDrive(velocity, turn);
  }
  //Mercy Arcade Drive allows us to smoothly control the robot
  public void mercyArcadeDrive(double joystickX, double joystickY){
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
        rightSpeed = -1;
      }
      if (leftSpeed < -1) {
        leftSpeed = -1;
      }
  
        if (RobotMap.climbClosedLoopMode ) {
          //closed loop
          double targetVelocity_UnitsPer100ms_left = leftSpeed * RobotMap.maximumVelocityFalcon;
          double targetVelocity_UnitsPer100ms_right = rightSpeed * RobotMap.maximumVelocityFalcon;
          climbLeftTalon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms_left);
          climbRightTalon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms_right);
        }
        
      else {
        //open loop
        climbLeftTalon.set(ControlMode.PercentOutput, leftSpeed);
        climbRightTalon.set(ControlMode.PercentOutput, rightSpeed);
      }
    }

      public void sizzleClimbHold(){
        double leftMotorPercent = climbLeftTalon.getMotorOutputPercent();
        double rightMotorPercent = climbRightTalon.getMotorOutputPercent();
        
        climbLeftTalon.configVoltageCompSaturation(12.0);
        climbRightTalon.configVoltageCompSaturation(12.0);

        climbLeftTalon.enableVoltageCompensation(true);
        climbRightTalon.enableVoltageCompensation(true);

        climbLeftTalon.set(ControlMode.PercentOutput, leftMotorPercent);
        climbRightTalon.set(ControlMode.PercentOutput, rightMotorPercent);
      }

      public void karenaNotArcadeDrive(double joystickX, double joystickY){
        //forward becomes postive
        //joystickY *= -1;
        joystickX *= -1;
        climbLeftTalon.set(ControlMode.PercentOutput, joystickX);
        climbRightTalon.set(ControlMode.PercentOutput, joystickY);
      }

      public void karenaArcadeDrive(double joystickX, double joystickY){
        //forward becomes postive
        joystickY *= -1;

        //System.out.println("ElevatorX = " + joystickX + "ElevatorY = " + joystickY);

        //The java.lang.Math.asin() returns the arc sine of an angle in between -pi/2 and pi/2.
        //double radiusPower = Math.sqrt(joystickX*joystickX + joystickY*joystickY);
        double radiusPower = Math.hypot(joystickX, joystickY);
        if (radiusPower > 0.3){
          //sin value range is from -0.5 pi to 0.5 pi
          //double initAngle = Math.atan2(joystickX, joystickY);
          double initAngle = Math.asin(Math.abs(joystickY)/radiusPower);
          double initAngleDegree = initAngle * RobotMap.radianConversionToDegree;
          if(joystickX > 0 && joystickY > 0  ) {
            // do nothing
          }
          else if(joystickX < 0 && joystickY > 0  ) {
            initAngleDegree = 180 - initAngleDegree;
          } 
          else if(joystickX < 0 && joystickY < 0  ) {
            initAngleDegree = 180 + initAngleDegree;
          } 
          else if(joystickX > 0 && joystickY < 0  ) {
            initAngleDegree = 360 - initAngleDegree;
          } 
          else if (isAboutEqual(joystickY, 0.0) && joystickX > 0 ) {
            initAngleDegree = 0;
          }
          else if (isAboutEqual(joystickX, 0.0) && joystickY > 0 ) {            
            initAngleDegree = 90;
          }
          else if (isAboutEqual(joystickY, 0.0) && joystickX < 0 ) {            
            initAngleDegree = 180;
          }
          else if (isAboutEqual(joystickX, 0.0) && joystickY < 0 ) {            
            initAngleDegree = 270;
          }

          System.out.println("Angle = " + initAngleDegree);

          if (initAngleDegree > (-1 * RobotMap.angleDeadBand) && initAngleDegree < RobotMap.angleDeadBand){
            //joystick is shifted to the right within angle deadband, does nothing
            climbLeftTalon.set(ControlMode.PercentOutput, 0);
            climbRightTalon.set(ControlMode.PercentOutput, 0);
          }
          else if (initAngleDegree > RobotMap.angleDeadBand && initAngleDegree < (90 - RobotMap.angleDeadBand)){
            //joystick is shifted to ther right above angle deadband, only the right arm moves up
            climbLeftTalon.set(ControlMode.PercentOutput, 0);
            climbRightTalon.set(ControlMode.PercentOutput,(-1 * RobotMap.climbSpeed));
          }
          else if (initAngleDegree > (90 - RobotMap.angleDeadBand) && initAngleDegree < (90 + RobotMap.angleDeadBand)){
            //shift upwards within deadband, both arms move up
            climbLeftTalon.set(ControlMode.PercentOutput, RobotMap.climbSpeed);
            climbRightTalon.set(ControlMode.PercentOutput, (-1 * RobotMap.climbSpeed));
          }
          else if (initAngleDegree > (90 + RobotMap.angleDeadBand) && initAngleDegree < (180 - RobotMap.angleDeadBand)){
            //shifted to the left within deadband, where only the left arm moves up
            climbLeftTalon.set(ControlMode.PercentOutput, RobotMap.climbSpeed);
            climbRightTalon.set(ControlMode.PercentOutput, 0);
          }
          else if (initAngleDegree > (180 - RobotMap.angleDeadBand) && initAngleDegree < (180 + RobotMap.angleDeadBand)){
            //shift to the left, within deadband so it does nothing
            climbLeftTalon.set(ControlMode.PercentOutput, 0);
            climbRightTalon.set(ControlMode.PercentOutput, 0);
          }
          else if (initAngleDegree > (180 + RobotMap.angleDeadBand) && initAngleDegree < (270 - RobotMap.angleDeadBand)){
            //shift downwards to the left outside the deadband, so only the left arm moves down
            climbLeftTalon.set(ControlMode.PercentOutput,  (-1 * RobotMap.climbSpeed));
            climbRightTalon.set(ControlMode.PercentOutput, 0);
          }
          else if (initAngleDegree > (270 - RobotMap.angleDeadBand) && initAngleDegree < (270 + RobotMap.angleDeadBand)){
            //shift downwards within deadband, so both arms move downwards
            climbLeftTalon.set(ControlMode.PercentOutput, (-1 * RobotMap.climbSpeed));
            climbRightTalon.set(ControlMode.PercentOutput, RobotMap.climbSpeed);
          }
          else if (initAngleDegree > (270 + RobotMap.angleDeadBand) && initAngleDegree < (360 - RobotMap.angleDeadBand)){
            //shift downwards within deadband, so only the right arm moves down
            climbLeftTalon.set(ControlMode.PercentOutput, 0);
            climbRightTalon.set(ControlMode.PercentOutput, RobotMap.climbSpeed);
          }

        }
        else {
          climbLeftTalon.set(ControlMode.PercentOutput, 0);
          climbRightTalon.set(ControlMode.PercentOutput, 0);
        }
    
    /*Convert the initial (x,y) coordinates to polar coordinates.
    Rotate them by 45 degrees.
  Convert the polar coordinates back to cartesian.
  Rescale the new coordinates to -1.0/+1.0.
  Clamp the new values to -1.0/+1.0.
  This assumes the initial (x,y) coordinates are in the -1.0/+1.0 range. The side of the inner square will always be 
  equal to l * sqrt(2)/2, so step 4 is just about multiplying the values by sqrt(2). */
  
  }

  void zeroSensors() {
    climbLeftTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.kTimeoutMs);
    climbRightTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.kTimeoutMs);
    System.out.println("[Quadrature Encoders] All drive sensors are zeroed.\n");
  }

  boolean isAboutEqual(double input, double to) {
     boolean ret = false;
     double diff =  Math.abs(input) - Math.abs(to);
     if (Math.abs(diff) < 0.001) {
        ret = true;
     }
     return ret;
  }
}
