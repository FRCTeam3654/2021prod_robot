// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.wpilibj.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.geometry.Translation2d;

//import java.util.List;

//import frc.robot.commands.NewRunMotionProfile;
//import frc.robot.commands.RunGalacticSearchARed;
//import frc.robot.commands.RunGalacticSearchABlue;
//import edu.wpi.first.wpilibj.geometry.Pose2d;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.RobotOdometry;
//import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunGalacticSearchB extends CommandBase {
  public boolean galacticRedB;

  NetworkTable mercyLimelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry MercyLimelightx = mercyLimelightTable.getEntry("tx");
  NetworkTableEntry MercyLimelighty = mercyLimelightTable.getEntry("ty");
  NetworkTableEntry MercyLimelightArea = mercyLimelightTable.getEntry("ta");
  private double locationX;
  private double locationY;
  private double locationA;
  private double startTimeLimelight = 0;
  private RobotOdometry odometry;
  private Drive driveTrain;
  private boolean runOnce = false;
  private Command runGalacticSearchBRed;
  private Command runGalacticSearchBBlue;
  
  public RunGalacticSearchB(RobotOdometry odometry, Drive driveTrain) {
    this.odometry = odometry;
    this.driveTrain = driveTrain;
  }


  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    startTimeLimelight = Timer.getFPGATimestamp();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //1 is force off
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1); //setting the pipeline to 1 for finding power cell
  
    runGalacticSearchBRed = new RunGalacticSearchBRed(odometry, driveTrain);
    runGalacticSearchBBlue = new RunGalacticSearchBBlue(odometry, driveTrain);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if(runOnce == false) {
      //read values periodically
      locationX = MercyLimelightx.getDouble(0.0);
      locationY = MercyLimelighty.getDouble(0.0);
      locationA = MercyLimelightArea.getDouble(0.0);

      //post to smart dashboard periodically
      SmartDashboard.putNumber("LimelightX", locationX);
      SmartDashboard.putNumber("LimelightY", locationY);
      SmartDashboard.putNumber("LimelightArea", locationA);

      if(startTimeLimelight + 0.25 < Timer.getFPGATimestamp()) {
        if(locationY < -2 && locationA > 0.2){
        galacticRedB = true;
        }else {
        galacticRedB = false;
        }
        SmartDashboard.putBoolean("GalacticRedB", galacticRedB);
        if(galacticRedB == true){
          runGalacticSearchBRed.schedule();
        } else { 
          runGalacticSearchBBlue.schedule();
        }
        runOnce = true;
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
  return false;
}

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //1 is force off LED - required by FRC
    runGalacticSearchBRed.cancel();
    runGalacticSearchBBlue.cancel();
  } 
}
