// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LatencyData;
import com.ctre.phoenix.sensors.PigeonIMU;

public class RobotOdometry extends SubsystemBase {
  // How many historical data points to keep. Multiply by 20ms to get time
  private static final int latencyDataPoints = 20;

  private Drive driveTrain;
  //private DriveSubsystem driveTrain;
  private PigeonIMU pigeon;
  private DifferentialDriveOdometry driveOdometry;
  private LatencyData xData = new LatencyData(latencyDataPoints);
  private LatencyData yData = new LatencyData(latencyDataPoints);

  private double baseLeftDistance;
  private double baseRightDistance;

  /**
   * Creates a new RobotOdometry. Commands can require this subsystem to prevent
   * the position from being changed by anything other than normal dead reckoning
   * while the command is running.
   * 
   * Initial position is (0,0) and 0 degrees
   */
  //public RobotOdometry(DriveSubsystem driveTrain, PigeonIMU pigeon) {
  public RobotOdometry(Drive driveTrain, PigeonIMU pigeon) {
    this.driveTrain = driveTrain;
    this.pigeon = pigeon;
    driveOdometry = new DifferentialDriveOdometry(getCurrentRotation());
    resetBaseDistances();
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  private Pose2d updateOdometry() {
    Pose2d pose = driveOdometry.update(getCurrentRotation(), driveTrain.getDistanceLeft() - baseLeftDistance,
        driveTrain.getDistanceRight() - baseRightDistance);
    Translation2d translation = pose.getTranslation();
    xData.addDataPoint(translation.getX());
    yData.addDataPoint(translation.getY());
    return pose;
  }

  // IEEERemainder = dividend - (divisor * Math.Round(dividend / divisor))  
 // IEEEremainder(double dividend, double divisor)
 //  f1 – f2 x n, where n is the mathematical integer closest to the exact mathematical value of the quotient f1/f2, and if two mathematical integers are equally close to f1/f2, then n is the integer that is even.
 public double getYaw() {
   return driveTrain.getYaw();
}


//  Positive rotations are counterclockwise.  it  represents the robot’s rotation relative to an axis on a 2-dimensional coordinate system.

//  Pigeon is different from the sample code IMU *** No need Negate pigeon angle****: 
//  right side is negative Y, left side is postive Y
public Rotation2d getHeading() {
   return driveTrain.getHeading();
}


// https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-odometry.html?highlight=rotation2d#creating-the-odometry-object 
//  As your robot turns to the left, your gyroscope angle should increase. By default, WPILib gyros exhibit the opposite behavior, so you should negate the gyro angle.
  private Rotation2d getCurrentRotation() {
    return Rotation2d.fromDegrees(getYaw() );
  }

  /**
   * Resets baseLeftDistance and baseRightDistance to the current position so
   * distances reported to the odometry class are 0.
   */
  private void resetBaseDistances() {
    baseLeftDistance = driveTrain.getDistanceLeft();
    baseRightDistance = driveTrain.getDistanceRight();
  }

  /**
   * Gets the robot's current pose.
   */
  public Pose2d getCurrentPose() {
    return driveOdometry.getPoseMeters();
  }

  /**
   * Sets the robot's current pose to the given x/y/angle.
   * 
   * @param x     The x coordinate
   * @param y     The y coordinate
   * @param angle The rotation component
   */
  public void setPosition(double x, double y, Rotation2d angle) {
    setPosition(new Pose2d(x, y, angle));
  }

  /**
   * Sets the robot's current pose to the given Pose2d.
   * 
   * @param position The position (both translation and rotation)
   */
  public void setPosition(Pose2d position) {
    driveOdometry.resetPosition(position, getCurrentRotation());
    resetBaseDistances();
  }

  /**
   * Adjusts the robot's position using the provided translation at the timestamp
   * given. The data since then is kept and the rotation component of the pose is
   * unchanged.
   * 
   * @param position  The translation component of the position
   * @param timestamp The timestamp the data is from (FPGA time)
   */
  public void setPosition(Translation2d position, double timestamp) {
    setPosition(position.getX(), position.getY(), timestamp);
  }

  /**
   * Adjusts the robot's position using the provided x and y at the timestamp
   * given. The data since then is kept and the rotation component of the pose is
   * unchanged.
   * 
   * @param x         The x coordinate
   * @param y         The y coordinate
   * @param timestamp The timestamp the data is from (FPGA time)
   */
  public void setPosition(double x, double y, double timestamp) {
    // Since this resets the odometry state make sure the numbers that are not being
    // changed are up to date to avoid losing changes that happened in between the
    // last update and this method call.
    updateOdometry();
    // This uses xData and yData to perform latency correction on the x/y portion of
    // the provided position
    xData.addCorrectedData(x, timestamp);
    yData.addCorrectedData(y, timestamp);
    driveOdometry.resetPosition(
        new Pose2d(xData.getCurrentPoint(), yData.getCurrentPoint(), getCurrentPose().getRotation()),
        getCurrentRotation());
    resetBaseDistances();
  }

  /**
   * Adjusts the robot's position using the provided x at the timestamp given. The
   * data since then is kept and the y and rotation components of the pose is
   * unchanged.
   * 
   * @param x         The x coordinate
   * @param timestamp The timestamp the data is from (FPGA time)
   */
  public void setX(double x, double timestamp) {
    updateOdometry();
    xData.addCorrectedData(x, timestamp);
    driveOdometry.resetPosition(
        new Pose2d(xData.getCurrentPoint(), getCurrentPose().getTranslation().getY(), getCurrentPose().getRotation()),
        getCurrentRotation());
    resetBaseDistances();
  }

  /**
   * Adjusts the robot's position using the provided y at the timestamp given. The
   * data since then is kept and the x and rotation components of the pose is
   * unchanged.
   * 
   * @param y         The y coordinate
   * @param timestamp The timestamp the data is from (FPGA time)
   */
  public void setY(double y, double timestamp) {
    updateOdometry();
    yData.addCorrectedData(y, timestamp);
    driveOdometry.resetPosition(
        new Pose2d(getCurrentPose().getTranslation().getX(), yData.getCurrentPoint(), getCurrentPose().getRotation()),
        getCurrentRotation());
    resetBaseDistances();
  }

  /**
   * Sets the robot's current rotation without affecting the translation component
   * of the pose.
   * 
   * @param rotation The rotation component of the pose
   */
  public void setRotation(Rotation2d rotation) {
    updateOdometry();
    Translation2d currentTranslation = getCurrentPose().getTranslation();
    driveOdometry.resetPosition(new Pose2d(currentTranslation, rotation), getCurrentRotation());
    resetBaseDistances();
  }




  public void resetOdometry() {
    driveOdometry.resetPosition(new Pose2d(), getHeading());
  }
  public void resetOdometry(Pose2d pose) {
    driveTrain.resetEncoders();
    driveOdometry.resetPosition(pose, getHeading());
  }

  //Resets the robot's position on the field.
  //You NEED to reset your encoders (to zero) when calling this method.
  //The gyroscope angle does not need to be reset here on the user's robot code. The library automatically takes care of offsetting the gyro angle.
}
