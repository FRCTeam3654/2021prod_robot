// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 1;
    public static final int kLeftMotor2Port = 3;
    public static final int kRightMotor1Port = 2;
    public static final int kRightMotor2Port = 4;

    public static final int[] kLeftEncoderPorts = new int[] {0, 1};
    public static final int[] kRightEncoderPorts = new int[] {2, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final double kTrackwidthMeters = 0.62; //0.62, 0.64
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 2048;
    public static final double encoderTicksPerRev = 2048; // added by Team 3654
    public static final double kWheelDiameterMeters = 0.1524;
    public static final double kWheelCircumferenceMeter = 0.4785; // pi x 6.0 x 2.54 /100 added by Team 3654
    
    //public static final double kGearing = 10.71  ;   // added by Team 3654
    public static final double gearRatio = 1 / 10.71; // added by Team 3654
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) * gearRatio / (double) kEncoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.618;
    public static final double kvVoltSecondsPerMeter = 2.31;// from 2.31,  1.2
    public static final double kaVoltSecondsSquaredPerMeter = 0.173;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 2.2; // 7.0
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.3;  //3, 2.0 , 1.7
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.3; //3, 2.0 , 1.7
    public static final double kMaxCentripetalAcceleration = 0.9 * kMaxAccelerationMetersPerSecondSquared;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}