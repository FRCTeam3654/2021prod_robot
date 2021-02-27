// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// code from Team 5010 https://github.com/FRC5010/InfiniteRecharge-2020/blob/master/InfiniteRecharge/src/main/java/frc/robot/commands/StartStopTimer.java

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;



public class StartStopTimer extends CommandBase {
  static long startTime = 0;
  static long currTime = 0;
  
    /** Creates a new StartStopTimer. */
    static {
      ShuffleboardLayout layout = Shuffleboard.getTab(Constants.DriveConstants.SBTabDriverDisplay)
      .getLayout("Timer", BuiltInLayouts.kList).withPosition(6, 0).withSize(1, 1);
      layout.addNumber("Time", StartStopTimer::getTime).withPosition(0, 1);
      layout.addNumber("TimeFromInitializer", StartStopTimer::getTimeFromInitializer).withPosition(0, 2);
      layout.addNumber("TimeDiff(ms)", StartStopTimer::getTimeDiff).withPosition(0, 3);
    }
    public StartStopTimer() {
      // Use addRequirements() here to declare subsystem dependencies.
    }
  
    static double getTime() {
      return (currTime - startTime)/1000.0;
    }

    static double getTimeFromInitializer() {
      return (currTime - Robot.m_auto_starttime)/1000.0;
    }

    static double getTimeDiff() {
      return (startTime - Robot.m_auto_starttime);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      startTime = currTime = System.currentTimeMillis();
      
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      currTime = System.currentTimeMillis();
      SmartDashboard.putNumber("Timer", currTime-startTime);
      
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }