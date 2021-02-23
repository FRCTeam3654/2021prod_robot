/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Robot;

public class AutonomousCommandGroup extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
    //:)

    public static boolean stopAutonomousCommandGroup = false;
    private int autoMode; //what route the robo will take
    // 0 = DriveStraight
    // 1 = CenterDeploy
    // 2 = RightDeploy
    // 3 = LeftDeploy
    // 4 = LeftTurn
    // 5 = CenterTurn
    // 6 = RightTurn
    // 7 = Def30
    // 8 = Def90
  
    
  //This constructor is necessary for Robot Init because it does not have a parameter
    public AutonomousCommandGroup() {
       //String autonomous = SmartDashboard.getString("Autonomous", "Default");
       int autonomous =  (int) Math.round(SmartDashboard.getNumber("Autonomous", 1.0)); 

       
     autonomous = 4;

        if (autonomous == 0 ) {  //GalacticA
          addCommands(new AutonomousDriveCommand(2));
          /*  
          addSequential(new AutonomousDriveBackwardsCommand());
          //addSequential(new WaitCommand(2.0));// to replace it by ballshooter
          addSequential(new BallShooterCommand(1));
          addSequential(new AutonomousDriveCommand());
          //addSequential(new BallPickUpCommand(1));
          */
         
        } else if (autonomous == 1){ //GalacticB
          addCommands(new AutonomousDriveCommand(2));
      
          //addSequential(new AutonomousDriveBackwardsCommand());
          //addSequential(new WaitCommand(2.0));// to replace it by ballshooter
          //addSequential(new BallShooterCommand(1));
         // addSequential(new AutonomousDriveCommand());


        } else if (autonomous == 2){ //Bounce Path
          addCommands(new AutonomousDriveCommand(2));
/*
          addSequential(new AutonomousDriveBackwardsCommand());
          addSequential(new WaitCommand(2.0));// to replace it by ballshooter
          addSequential(new AutonomousDriveCommand());
          addSequential(new BallPickUpCommand(1));
*/
         // addSequential(new MotionMagicDriveCommand(1.2, true));
         //addSequential(new AutonomousDriveBackwardsCommand()); // test path

        } else if (autonomous == 3){ //Slalom
          addCommands(new AutonomousDriveCommand(2)); 
          /*
          addParallel(new BallPickUpCommand(1));
          addSequential(new MotionMagicDriveCommand(2.629));
          addSequential(new MotionMagicDriveCommand(3.5434));
          addSequential(new WaitCommand(1.0));
          addParallel(new BallPickUpCommand(2, 1.0));
          addSequential(new MotionMagicDriveCommand(-3.5434));
          addSequential(new AutonomousDriveBackwardsCommand());
          addSequential(new BallShooterCommand(1));
          addSequential(new AutonomousDriveCommand());
          addParallel(new BallPickUpCommand(1));
          addSequential(new MotionMagicDriveCommand(4.438));
          
          
          addSequential(new MotionMagicDriveCommand(1.2, true));
          */
        } else if (autonomous == 4){ //Barrel
          addCommands(new AutonomousDriveCommand(2));

        // addSequential(new MotionMagicDriveCommand(2.0, false, false));
         //addSequential(new MotionMagicDriveCommand(2.0, false, true));
         
        // addSequential(new WaitCommand(1.0)); // test
        // addSequential(new TurnSmallDegreeCommand(10.0, true));
        // Robot.drive.setRightTalonFXInvert(true);
        // Robot.drive.setLeftTalonFXInvert(false);
        


         /* addSequential(new AutonomousDriveBackwardsCommand(1)); //first motion profile path of mode 4
          addSequential(new BallShooterCommand(1)); //1 means autonomous mode
          //addSequential(new WaitCommand(2.0));// to replace it by ballshooter
          //addParallel(new BallPickUpCommand(1));
          addSequential(new AutonomousDriveCommand(2)); //second motion profile path of mode 4
          addSequential(new MotionMagicDriveCommand(0.91, true));
          addSequential(new MotionMagicDriveCommand(0.91, true));
          */
          //addSequential(new AutonomousDriveBackwardsCommand(3)); //third motion profile path of mode 4
          //addSequential(new BallShooterCommand(1));
 /*       
          Robot.drive.zeroSensors();
          addParallel(new BallPickUpCommand(1));
          addSequential(new MotionMagicDriveCommand(2.629, true));     
          addSequential(new WaitCommand(3));
          addSequential(new MotionMagicDriveCommand(0.9, true));
          addSequential(new WaitCommand(3.0));
          addParallel(new BallPickUpCommand(2, 1.0));
          addSequential(new MotionMagicDriveCommand(-3.5, true));
*/
        } else if (autonomous == 5){ //CenterTurn
          //addParallel(new BallPickUpCommand(1));
          //addSequential(new MotionMagicDriveCommand(2.414));
          //addSequential(new WaitCommand(1.0));
          //addParallel(new BallPickUpCommand(2, 1.0));
          //addSequential(new MotionMagicDriveCommand(0.762));
          //addSequential(new MotionMagicDriveCommand(1.2, true));
          addCommands(new AutonomousDriveBackwardsCommand()); // test path
        } else if (autonomous == 6){ //RightTurn
          //addParallel(new BallPickUpCommand(1));
          //addSequential(new MotionMagicDriveCommand(2.8254));
          //addSequential(new WaitCommand(1.0));
          //addParallel(new BallPickUpCommand(2, 1.0));
          //addSequential(new MotionMagicDriveCommand(1.884));
         // addSequential(new MotionMagicDriveCommand(1.2, true));
         addCommands(new AutonomousDriveBackwardsCommand()); // test path
        } else if (autonomous == 7){ //Def30
          //addSequential(new AutonomousDriveCommand());
         // addSequential(new MotionMagicDriveCommand(1.2, true));
         addCommands(new AutonomousDriveBackwardsCommand()); // test path
        }else if (autonomous == 8){ //Def90
          //addSequential(new AutonomousDriveCommand());
          //addSequential(new MotionMagicDriveCommand(1.2, true));
        }
     
    } 
}
