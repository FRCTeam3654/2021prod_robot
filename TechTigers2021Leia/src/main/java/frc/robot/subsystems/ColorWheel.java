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
//import frc.robot.commands.ColorWheelCommand;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
//import frc.robot.Robot;

public class ColorWheel extends SubsystemBase {
  private TalonSRX colorWheelTalon = new TalonSRX(RobotMap.colorWheelTalonID);
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 seeTheRainbow = new ColorSensorV3(i2cPort);
  private final ColorMatch rainbowMatcher = new ColorMatch();
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public final Color kBlueTarget = ColorMatch.makeColor(RobotMap.colorTargets[0][0], RobotMap.colorTargets[0][1], RobotMap.colorTargets[0][2]);
  public final Color kGreenTarget = ColorMatch.makeColor(RobotMap.colorTargets[1][0], RobotMap.colorTargets[1][1], RobotMap.colorTargets[1][2]);
  public final Color kRedTarget = ColorMatch.makeColor(RobotMap.colorTargets[2][0], RobotMap.colorTargets[2][1], RobotMap.colorTargets[2][2]);
  public final Color kYellowTarget = ColorMatch.makeColor(RobotMap.colorTargets[3][0], RobotMap.colorTargets[3][1], RobotMap.colorTargets[3][2]);

  @Override
  public void periodic() {
   
   
  }
  
  public ColorWheel() {
    colorWheelTalon.configFactoryDefault();
    colorWheelTalon.setNeutralMode(NeutralMode.Brake);
    //configure sensor source for primary PID
    //colorWheelTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, RobotMap.kPIDLoopIDx, RobotMap.kTimeoutMs);
    colorWheelTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.kPIDLoopIDx, RobotMap.kTimeoutMs);
    //set deadband to super small 0.001 (.1%)
    colorWheelTalon.configNeutralDeadband(0.001, RobotMap.kTimeoutMs);
    /* configure Talson SRX utput and sensor direction occordingly invert motor to
    *have green LEDs when driving Talon Forward / requesting positive utput phase sensor
    *to have positive increment when driving Talon Forward (Green LED) */
    colorWheelTalon.setSensorPhase(false);
    //making this true or false does the same thing
    colorWheelTalon.setInverted(true);
    /* set relevant frame periods to be at least as fast as periodic rate */
    colorWheelTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.kTimeoutMs);
    colorWheelTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.kTimeoutMs);
    /* set the peak and nominal outputs */
    colorWheelTalon.configNominalOutputForward(0, RobotMap.kTimeoutMs);
    colorWheelTalon.configNominalOutputReverse(0, RobotMap.kTimeoutMs);
    colorWheelTalon.configPeakOutputForward(1, RobotMap.kTimeoutMs);
    colorWheelTalon.configPeakOutputReverse(-1, RobotMap.kTimeoutMs);
    /* set the Motion Magic gains in slot0 - see documentation */
    colorWheelTalon.config_kF(RobotMap.kColorWheelSlotIDx, RobotMap.kGains.kF, RobotMap.kTimeoutMs);
    colorWheelTalon.config_kP(RobotMap.kColorWheelSlotIDx, RobotMap.kGains.kP, RobotMap.kTimeoutMs);
    colorWheelTalon.config_kI(RobotMap.kColorWheelSlotIDx, RobotMap.kGains.kI, RobotMap.kTimeoutMs);
    colorWheelTalon.config_kD(RobotMap.kColorWheelSlotIDx, RobotMap.kGains.kD, RobotMap.kTimeoutMs);

    colorWheelTalon.selectProfileSlot(RobotMap.kColorWheelSlotIDx, RobotMap.kPIDLoopIDx);
    /* set acceleration and vcruise velocity - see documentation */
    //numbers should be experimentally derived once we have the color wheel system in place
    colorWheelTalon.configMotionCruiseVelocity(RobotMap.colorWheelCruiseVelocity, RobotMap.kTimeoutMs);
    colorWheelTalon.configMotionAcceleration(RobotMap.colorWheelAcceleration, RobotMap.kTimeoutMs);
    /* zero the sensor once on robot boot up*/
    colorWheelTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.kTimeoutMs);

    rainbowMatcher.addColorMatch(kBlueTarget);
    rainbowMatcher.addColorMatch(kGreenTarget);
    rainbowMatcher.addColorMatch(kRedTarget);
    rainbowMatcher.addColorMatch(kYellowTarget);
  }

  public void zeroSensor() {
    colorWheelTalon.setSelectedSensorPosition(0, RobotMap.kPIDLoopIDx, RobotMap.kTimeoutMs);
    System.out.println("[Color Wheel sensor is zeroed] \n");
  }
  public void spinColorWheel(double targetPos){
    //Robot.drive.setPositionClosedLoop(1000);
    colorWheelTalon.set(ControlMode.MotionMagic, targetPos);
    System.out.println(colorWheelTalon.getSelectedSensorVelocity(0) + ",  " + colorWheelTalon.getClosedLoopError(0) + ",  " + colorWheelTalon.getSelectedSensorPosition(0));
  }

  public int colorWheelTickCount(){
    return (int)colorWheelTalon.getSelectedSensorPosition();   
  }

  public void manualColorWheelSpin(double percentOutput){
    colorWheelTalon.set(ControlMode.PercentOutput, percentOutput);
  }
 
public int getRainbow(){
  Color detectedColor = seeTheRainbow.getColor();
  // Run the color match algorithm on our detected color
  String colorString;
  ColorMatchResult match = rainbowMatcher.matchClosestColor(detectedColor);
  int Rainbow;
    if (match.color == kBlueTarget) {
      colorString = "Blue";
      Rainbow = 1;
    } else if (match.color == kRedTarget) {
      colorString = "Red";
      Rainbow = 2;
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
      Rainbow = 3;
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
      Rainbow = 4;
    } else {
      colorString = "Unknown";
      Rainbow = 0;
    }
    if (match.confidence <= 0.95){
      colorString = "Unknown";
      Rainbow = 0;
    }
    //Blue = 1 Red = 2 Green = 3 Yellow = 4
    // Open Smart Dashboard or Shuffleboard to see the color detected by the sensor.
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
    return Rainbow;
  }

  public int colorMatching(){
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0)
    {
      switch (gameData.charAt(0))
      //Blue = 1 Red = 2 Green = 3 Yellow = 4
      {
        case 'B' :   //Blue case code
          return 1;
        case 'G' :   //Green case code
          return 3;
        case 'R' :   //Red case code
          return 2;
        case 'Y' :   //Yellow case code
         return 4;
        default :   //This is corrupt data
          return 0;
      }
    } else {
      return 0;  //Code for no data received yet
    }
  }

  public int colorTransfer(int gameColor){  
      switch (gameColor)
      //Blue = 1 Red = 2 Green = 3 Yellow = 4
      {
        case 1 :
          //Blue case code
          return 2;
        case 3 :
          //Green case code
          return 4;
        case 2 :
          //Red case code
          return 1;
        case 4 :
          //Yellow case code
         return 3;
        default :
          //This is corrupt data
          return 0;
      }
  }
}





