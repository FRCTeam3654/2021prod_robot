/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// imports 
package frc.robot;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatch;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

public class RobotMap {

static Preferences prefs; 

//DRIVE VALUES!
//test branch
public static double driveStraightProportion = 0.01;//0.02
public static double turnDegreeProportion = 0.0038;
public static double LimelightJoeyX = -0.007;
public static double turn90DegreeTimeout = 1.5;
public static double limeLightTimeout = 2;
public static double ballStorageTimerAndysVision = 2;
public static double autonomousVelocity = 0.5; 
public static double leftOverRightCompensation = .98;
public static double talonDriveAccelerationRate = 0.3654;
public static double ballStorageSpeed = -0.6;
public static double effectiveTurnWheelWidth = 0.64; // meter, measured by turning the robot

// ^^^ Must be experimentally derived

public static double driveDeadband = 0.05;
public static double autonomousTargetPos = 219340.8/2; //gear ratio is :  10.71:1  219340.8 is ten rounds of big wheel
public final static double kMeterToFalconSenorUnit = 45812.56; // measured effective wheel diameter is 6.0 inches
public final static int kSensorUnitsPerRotation = 2048;
public final static double kNeutralDeadband = 0.001; // testing MP

public static double leftPercentOutput = 0.9;
public static double rightPercentOutput = 0.9;
public static double nonTurboMultiplierTurn = 0.15;
public static double nonTurboMultiplierForward = 0.32; 

public static boolean driveClosedLoopMode = false;

//BUTTON/PORT NUMBERS!
public static int driverJoystickPort = 0;
public static int operatorJoystickPort = 1;

//DRIVE STICK
public static int turboButtonNumber = 1;
public static int driveStraightButtonNumber = 2;
public static int turnLeft90ButtonNumber = 3; 
public static int turnRight90ButtonNumber = 4; 
public static int turnLeft180ButtonNumber = 5; 
public static int turnRight180ButtonNumber = 6;
public static int limeLightButtonNumber = 8; 

//OPERATOR STICK
public static int colorWheelSpinButtonNumber = 9;
public static int climbLockLeftButtonNumber = 7;
public static int climbLockRightButtonNumber = 5;
public static int climbUnlockLeftButtonNumber = 8;
public static int climbUnlockRightButtonNumber = 6;
public static int ballFlushButtonNumber = 2;  
public static int colorSensorButtonNumber = 10; 
public static int pickUpButtonNumber = 1;
public static int ballShooterButtonNumber = 4; 
public static int turretButtonNumber = 3;

//TALONS/TALON ID NUMBERS!
public static int leftTalonMaster = 1;
public static int rightTalonMaster = 2;
public static int leftTalonSlave = 3;
public static int rightTalonSlave = 4;
public static int vinnieTalonNumber = 6;

public static int climbExtendTalonID = 10;
public static int climbLeftTalonID = 11;
public static int climbRightTalonID = 12;
public static int BallPickUpID = 6;
public static int BallShooterID = 8;
public static int BallShooterSlaveID = 5;
public static int pickUpArmTalonID = 13;
public static int turretTurningID = 17;

// Digital IO for the ballpickup sensor
public static int ArmStatusID = 0;

//public static int BallStorageID = 7;//7 on competition robot

//temp numbers cuz idk what i'm doinggggggg
public static int BallStorageID1 = 7;
public static int BallStorageID2 = 15;
//public static int BallStorageID3 = 16;
//public static int BallStorageID4 = 7;


public static int colorWheelTalonID = 9;

public static int analogDistanceSensorPort1 = 0; 
public static int digitalDistanceSensorPort2 = 0; 
public static int digitalDistanceSensorPort3 = 1; 
//public static int digitalDistanceSensorPort4 = 3; 
//public static int digitalDistanceSensorPort5 = 4; //oh yeah 

public static double joystickDeadBand = 0.08;

public static double autonomousTimeOut = 40; // used to be 7 second in normal auto mode but 2021 is different

public static double autonomousBallShooterTimeOut = 3;
public static double autonomousBallPickUpTimeOut = 4; // MICHELE WAS HERE

public static double motionMagicTimeOut = 40;// in regular, it should time out in 4 seconds

//falcon maximum velocity in native unit 
public static int maximumVelocityFalcon = 21000;
public static double radianConversionToDegree = 57.2958;

//BALL INTAKE/SHOOTER
public static double ballPickUpSpeed = 0.5;// used to 0.4
public static int solenoidIn = 3;
public static int solenoidOut = 2;
public static double shooterSpeed_nativeUnit = 10240; //3000 rpm
//^^ calculated by (desired rpm * 2048 / 60sec / 10)
public static double shooterSpeedTolerance = 200;

//COLOR WHEEL!
/**4096 * 25
*25 is minimum times small wheel should turn for big wheel turning 3 times
**/
public static int turretTickAmount = 20000; //82,944 ticks 1:81 gear ratio for half turn
public static int colorWheelSpinTickAmount = 102400;
public static int colorWheelCruiseVelocity = 4000;
public static int colorWheelAcceleration = 4000;
public static int colorSensorTickAmount = 4096; //for spinning the wheel 1/8 to change to an adjacent color
public static double colorSensorTimeout = 7;

/*public final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  public final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  public final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  public final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  blue, green, red, yellow*/
  public static double[][] colorTargets = {{0.144, 0.438, 0.417},{0.179, 0.572, 0.252},{0.468, 0.375, 0.157},{0.318, 0.547, 0.132}};

// --CLIMB--
public static int climbCruiseVelocity = 50000;
public static int climbAcceleration = 5000;
public static int climbMaxHeight = 40000;
public static int angleDeadBand = 10;
public static double climbSpeed = 0.2; //percent output

public static boolean climbClosedLoopMode = false;

public static int climbLockLeftSolenoidIn = 4;
public static int climbLockLeftSolenoidOut = 5;
public static int climbLockRightSolenoidIn = 0;
public static int climbLockRightSolenoidOut = 1;

//CLOSED LOOP!
//Constants used for color wheel PID Loop
//Closed loop for pick up arm
public static int pickUpArmSpinTickAmount = 16000;
public static int pickUpArmCruiseVelocity = 40000;
public static int pickUpArmAcceleration = 4000;

public static void getPreference(){
prefs = Preferences.getInstance();
driveClosedLoopMode = prefs.getBoolean("DriveClosedLoopMode", true);
}
/**
 * which PID slot to pull gains from. starting 2018, you can choose from 0,1,2 or 3.
 * only the first two (0,1) are visible in web-based
 * configuration
 */
public static final int kSlotIDx = 0; //default for drive
public static final int kColorWheelSlotIDx = 1;
public static final int kPickUpArmSlotIDx = 1;
public static final int kClimbSlotIDx = 2;
public static final int kShooterSlotIDx = 3;
public static final int kTurnAutonomousSlotIDx = 1;
public static final boolean kUseMotionProfileArc = false;

public final static int PID_PRIMARY = 0;
public final static int PID_TURN = 1;
public final static int REMOTE_0 = 0;
public final static int REMOTE_1 = 1;

/*these are the pid gains responsiveness to the control loop
*kF: 1023 represents toutput value to Talon at 100%, 7200 represents velocity units at 100% output
*                                                         kP,  kI,  kD,    kF,  Iz,  PeakOutput*/
//public final static Gains driveGainsVelocity = new Gains( 0.25, 0.0, 0.0, 1.015, 400, 1);
//public final static Gains driveGainsVelocity = new Gains( 0.095, 0.0, 0.0, 0.0451, 100, 1);
public final static Gains turnGainsVelocity = new Gains( 1.5, 0.0, 0.0, 0.000, 100, 1);
//public final static Gains climbGainsVelocity = new Gains( 0.25, 0.0, 0.0, 1.015, 400, 1);
//public final static Gains shooterGainsVelocity = new Gains( 0.03, 0.0, 0, 0.0451, 0, 0.5);
//public final static Gains kGains_MotProf = new Gains( 0.1, 0.0,  0.0, 0.0455,  400,  0.5 );
//public static int pidLoopTimeout = 30;

//public final static Gains driveGainsVelocity = new Gains( 0.0095, 0.0, 0.0, 0.0451, 100, 1); //0.3 used by motion profile as position loop
public final static Gains driveGainsVelocity = new Gains( 0.1, 0.0, 0.0, 0.0451, 100, 1); //0.3 used by motion profile as position loop

public final static Gains climbGainsVelocity = new Gains( 0.3, 0.0, 0.0, 0.0451, 400, 1);
public final static Gains shooterGainsVelocity = new Gains( 0.03, 0.0, 0, 0.0451, 0, 1);
public final static Gains turretGainsVelocity = new Gains( 0.03, 0.0, 0, 0.0451, 0, 1);
public final static Gains kGains_MotProf = new Gains( 0.001, 0.0,  0.0, 0.0451,  100,  1.0 );// p=0.00095
//static final Gains kGains = new Gains(0.03, 0.0, 0, 0.0451, 0, 0.5);
public static int pidLoopTimeout = 30;

/**
 * Talon SRX/ Victor SPX will supported miltiple (cascaded) PID Loops For 
 * now we just want the primary one.
 */
public static final int kPIDLoopIDx = 0;
public static final int pickUpArmPIDLoopIDx = 0;
/**
 * set to zero to skip waiting for confirmation, set to nonzero to wait
 *  wait and report to DS if action fails
 */
public static final int kTimeoutMs = 30;
public static final int pickUpArmTimeoutMs = 30;
/** --GAINs--
 * gains used to Motion Magic, to be adjusted acccordingly
 * Gains(kp, ki, kd, kf, izone, peak output);
 */
public static final Gains kGains = new Gains(0.2, 0.0, 0.0, 0.2, 0, 1.0);
public static final Gains pickUpArmGains = new Gains(0.2, 0.0, 0.0, 0.2, 0, 1.0);
//public static final Gains kGains = new Gains(0.095, 0.0, 0, 0.0451, 0, 0.25); // colorWheelTalon  use this

}

