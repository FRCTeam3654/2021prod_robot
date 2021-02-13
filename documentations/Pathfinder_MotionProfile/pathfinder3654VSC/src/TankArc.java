// https://github.com/JacisNonsense/Pathfinder/wiki/Pathfinder-for-FRC---Java


// https://www.chiefdelphi.com/forums/showthread.php?t=146303&page=2&highlight=motion+profile
/*
  	X+ is forward from where your robot starts.
	X- is backward from where your robot starts.
	Y+ is to the right of your robot where it starts.
	Y- is to the left of your robot where it starts.
	
	for driver straight 60 inches, use Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, 0); new Waypoint(60, 0, 0) ;}
	
	Three lessons learned today 5/28/2017
	1) THIS pathfinderleft/right profile may not match the robot left/right if robot's motor output is inverted.  need swap.  When I swapped them, the result match the curve
	2) wheelbase_width is much wider than actual physical width (0.4445): empirical is 0.8.  may account for some slippage
	3) use meter as unit to generate, then x 2.0897 to convert back from meter to rotations. because wheelbase is using meter, not rotation mentioned
 */


import java.io.File;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

//Create the Trajectory Configuration
//
//Arguments:
//Fit Method:          HERMITE_CUBIC or HERMITE_QUINTIC
//Sample Count:        SAMPLES_HIGH (100 000)
//                   SAMPLES_LOW  (10 000)
//                   SAMPLES_FAST (1 000)
//Time Step:           0.05 Seconds
//Max Velocity:        1.7 m/s
//Max Acceleration:    2.0 m/s/s
//Max Jerk:            60.0 m/s/s/s



public class TankArc {
	
	public static void main(String[] args) {
		
		   // System.setProperty("java.library.path", "C:\\Pathfinder\\wpilib\\user\\java\\lib\\pathfinderjava.lib");

    	
		   //String FILENAME = "C:\\Pathfinder\\mp_20ms_in_meter_frontcargo1_arc.csv";
	       String FILENAME = "C:\\Pathfinder\\autonomous\\mp_20ms_in_meter_arc.csv"; 
	       
	       
	       
	    	try {
	    	
	    	long starttime = System.nanoTime();
	    	
	    	int time_step_ms = 20;
	    	
	    	double wheelbaseinmeter =0.64;//0.652;// 0.635;//0.7;// 0.8; 0.55245 is 2021 robot
	    	
	    	
	    	//boolean useRotationAsUnit = false;
	    	//boolean useRotationAsUnit =  true;
	    	//double effective_wheel_diameter_in_inches = 5.645;//inches
	    	//double meter2RotationConversion = 100 / 3.14 / effective_wheel_diameter_in_inches / 2.54;
	    	double time_step = ((double)time_step_ms )/ 1000; // second,  = 5 ms
	    	
	    	//int wait_in_ms = 3000;
	    	
	    	int wait_in_ms = 0;
			
			
	    	int leftJumpDirection = 0; // -1: current Jaci's angel -360 ==> real correct value;   +1: current Jaci's angel+360 ==> real correct value, 0 no jump so far
	    	int rightJumpDirection = 0; // -1: current Jaci's angel -360 ==> real correct value;   +1: current Jaci's angel+360 ==> real correct value, 0 no jump so far
	    	
	    	
	    	double MAX_TURN_RATE_IN_20_MS = 20.0;
	    	
	    	
	    	//if(useRotationAsUnit  == false) {
	    	//	FILENAME = "C:\\Pathfinder\\mp_20ms_in_meter.csv";
	    	//}
		 
	    	
	    	

	    	// ---- use default meter/s as unit  and time_step_ms
	    	// ---- Need EXPERIMENT with max velocity and acceleration
	       //Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, time_step, 1.2, 1.2, 12); 
	       
	    	// slow
	    	//Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, time_step, 0.6, 0.7, 12.0); 
	       
	    	// faster good speed at~ 3 seconds 
	    	//Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, time_step, 1.5, 3, 12.0); 
	       
	    	
	    	// fasterer  
	    	//Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, time_step, 1.5, 3, 12.0); 
	       
	    	
	    	
	    	
	    	// test  
	    	//Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, time_step, 0.7, 1, 12.0); 
	       
	    
	    	Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, time_step, 1.3, 1.3, 30.0); 
		       
	    	//Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, time_step, 0.6, 1.2, 20.0); 
		    
	    	
	    	
	    	/*   Positive Y is on the left side
	    	 *   Positive X is the straight forward
	    	 *   Positive heading is Counter-clock-wise
	    	 * 
	    	 * 
	    	 */
	    	
	    	
	    	// nowing that Pathfinder paths shouldn�t go negative,
	    	
	    	// 2020 robot :  left side Y is positive
	    	
	    	
	    	//2021:   Glantic Search A:  tan(71.565)=3  (0,0,0) -> (1.524, 0.0, 0),  (0,0, Pathfinder.d2r(30) ) --> (1.524, -0.762, Pathfinder.d2r(30)),  (0,0, Pathfinder.d2r(30) ) ->(0.762, 2.286, Pathfinder.d2r(30)) 
	    	
	    	//        Barrel Path:  new Waypoint( 2.286, 0, Pathfinder.d2r(0) ), new Waypoint(2.99, -0.762, Pathfinder.d2r(-90)) , new Waypoint(2.286, -1.524, Pathfinder.d2r(-180)),new Waypoint(1.524, -0.762, Pathfinder.d2r(-270)) , new Waypoint(2.286, 0, Pathfinder.d2r(-360))
	    	
	        Waypoint[] points = new Waypoint[] {

						//Barrel Path
						/*
						//Barrel Path first loop
						new Waypoint(0, 0, 0),
	        		    new Waypoint( 1.05, 0, Pathfinder.d2r(0) ),
	        		    new Waypoint(1.81, -0.762, Pathfinder.d2r(-90)) ,
	        		    new Waypoint(1.05, -1.524, Pathfinder.d2r(-180)),
	        		    new Waypoint(0.286, -0.762, Pathfinder.d2r(-270)) ,
						new Waypoint(1.05, 0, Pathfinder.d2r(-360))
						//Barrel Path second loop
	        		    new Waypoint(0, 0, 0),
						new Waypoint(1.33, 0, Pathfinder.d2r(0)),
	        		    new Waypoint(2.1, 0.762, Pathfinder.d2r(90)),
	        		    new Waypoint(1.33, 1.524, Pathfinder.d2r(180)),
	        		    new Waypoint(0.57, 0.762, Pathfinder.d2r(270)),
						new Waypoint(0.57, 0, Pathfinder.d2r(270))
						//Barrel Path third loop
						new Waypoint(0, 0, 0),
						new Waypoint(1, 0, 0),
						new Waypoint(1.85, 0.762, Pathfinder.d2r(90)),
						new Waypoint(1.0, 1.524, Pathfinder.d2r(180)) ,
						new Waypoint(0, 1.524, Pathfinder.d2r(180)),
						*/

						// Barrel Path full path
						
						new Waypoint(0, 0, 0),
	        		    new Waypoint( 3.05, 0, Pathfinder.d2r(0) ),
	        		    new Waypoint(3.81, -0.762, Pathfinder.d2r(-90)) ,
	        		    new Waypoint(3.05, -1.524, Pathfinder.d2r(-180)),
	        		    new Waypoint(2.286, -0.762, Pathfinder.d2r(-270)) ,
						new Waypoint(3.05, 0, Pathfinder.d2r(-360)),
	        		    new Waypoint(5.33, 0, Pathfinder.d2r(-360)),
	        		    new Waypoint(6.1, 0.762, Pathfinder.d2r(-270)),
						new Waypoint(5.33, 1.524, Pathfinder.d2r(-180)),
						
	        		    new Waypoint(4.57, 0.762, Pathfinder.d2r(-90)),
	        		    new Waypoint(6.86, -1.524, Pathfinder.d2r(0)),
	        		    new Waypoint(7.62, -0.762, Pathfinder.d2r(90)) ,
						new Waypoint(6.86, 0, Pathfinder.d2r(180)),
						new Waypoint(0, 0,Pathfinder.d2r(180))
						// end of working Barrel Path
						
						
					/*		
					// test
						new Waypoint(0, 0, 0),
	        		    new Waypoint( 1.05, 0, Pathfinder.d2r(0) ),
	        		    new Waypoint(1.81, -0.762, Pathfinder.d2r(-90)) ,
	        		    new Waypoint(1.05, -1.524, Pathfinder.d2r(-180)),
	        		    new Waypoint(0.286, -0.762, Pathfinder.d2r(-270)) ,
						new Waypoint(1.05, 0, Pathfinder.d2r(-360)),
	        		    new Waypoint(3.33, 0, Pathfinder.d2r(-360)),
	        		    new Waypoint(4.1, 0.762, Pathfinder.d2r(-270)),
						new Waypoint(3.33, 1.524, Pathfinder.d2r(-180))
					*/
						/*new Waypoint(2.57, 0.762, Pathfinder.d2r(-90)),
	        		    new Waypoint(4.86, -1.524, Pathfinder.d2r(0)),
	        		    new Waypoint(5.62, -0.762, Pathfinder.d2r(90)) ,
						new Waypoint(4.86, 0, Pathfinder.d2r(180)),
						new Waypoint(0, 0,Pathfinder.d2r(180))
						*/



						/*
						//Slalom Path
						new Waypoint(0, 0, 0),
						new Waypoint(1.829, 0.762,Pathfinder.d2r(60)),
						new Waypoint(3.810,	1.524,Pathfinder.d2r(0)),
						new Waypoint(6.096, 0.762,Pathfinder.d2r(-45)),
						new Waypoint(6.858, 0,Pathfinder.d2r(0)),
						new Waypoint(7.620, 0.762,Pathfinder.d2r(90)),
						new Waypoint(6.858, 1.524,Pathfinder.d2r(180)),
						new Waypoint(6.096, 0.762,Pathfinder.d2r(235)),
						new Waypoint(3.810,	0.000, Pathfinder.d2r(180)),
						new Waypoint(1.829, 0.762,Pathfinder.d2r(120)),
						new Waypoint(0, 1.524,Pathfinder.d2r(180))
						*/
				

						/*
						//Bounce Path Sequence 1
						new Waypoint(0, 0, 0),
						new Waypoint(1.295, 0.762, Pathfinder.d2r(65)),
						new Waypoint(1.524, 1.524, Pathfinder.d2r(90))
						*//*
						//Bounce Path Sequence 2
						new Waypoint(0, 0, 0),						
						new Waypoint(2.286, 0.762, Pathfinder.d2r(30)),
						new Waypoint(3.048, 1.524, Pathfinder.d2r(90)),
						new Waypoint(2.286, 2.286, Pathfinder.d2r(180)),
						new Waypoint(0, 2.286, Pathfinder.d2r(180))
						*//*
						//Bounce Path Sequence 3
						new Waypoint(0, 0, 0),						
						new Waypoint(2.286, 0.076, Pathfinder.d2r(15)),
						new Waypoint(3.048, 0.762, Pathfinder.d2r(90)),
						new Waypoint(3.048, 1.524, Pathfinder.d2r(90)),
						new Waypoint(2.286, 2.210, Pathfinder.d2r(165)),
						new Waypoint(0, 2.286, Pathfinder.d2r(180))
						*//*
						//Bounce Path Sequence 4
						new Waypoint(0, 0, 0),
						new Waypoint(1.524, 1.524, Pathfinder.d2r(90))
						*/

						/*
						//Galactic Search Path A 
						//red 
						new Waypoint(0, 0, 0),
						new Waypoint(1.905, 0,Pathfinder.d2r(-30)),						
						new Waypoint(3.429, -0.762,Pathfinder.d2r(0)),
						new Waypoint(4.191, 1.524,Pathfinder.d2r(30)),
						new Waypoint(8.382, 1.524,Pathfinder.d2r(0))
						*/
					
						/*
						//blue
						new Waypoint(0, 0, 0),
						new Waypoint(6.096, 0,Pathfinder.d2r(-90)),
						new Waypoint(6.096, -1.524,Pathfinder.d2r(-180)),
						new Waypoint(3.810, -1.524,Pathfinder.d2r(90)),
						new Waypoint(3.810, 0.762,Pathfinder.d2r(0)),
						new Waypoint(4.572, 0.762,Pathfinder.d2r(0)),
						new Waypoint(7.620, 0.762,Pathfinder.d2r(0))
						*/



						/*
						//Galactic Search Path B
						//red
						new Waypoint(0, 0, 0),
						new Waypoint(1.905, 0, 0),						
						new Waypoint(3.429, -1.524, 0),
						new Waypoint(4.953, 0, 0),
						new Waypoint(8.382, 0, 0)
						*//*
						//blue
						new Waypoint(0, 0, 0),
						new Waypoint(4.191, -1.524, 0),						
						new Waypoint(5.715, 0, 0),
						new Waypoint(7.239, -1.524, 0),
						new Waypoint(8.382, -1.524, 0)
						*/




	          		    
	        };

	        Trajectory trajectory = Pathfinder.generate(points, config);

	        // @param wheelbase_width   The width (in meters) between the individual sides of the drivebase
	        // Wheelbase Width = 0.5m
	        // Our sister bot is 19 inches ( 0.4826 m) , breadboard is 17.5 inches (0.4445m)
	        // PEOPEL empirically found the base is larger than physical dimension 
	       // TankModifier modifier = new TankModifier(trajectory).modify(0.4445); // get only about 30 degree or so
	        TankModifier modifier = new TankModifier(trajectory).modify(wheelbaseinmeter); // get only about 30 degree or so
	        
	        
	        
	        // Do something with the new Trajectories...
	        Trajectory left = modifier.getLeftTrajectory(); 
	        Trajectory right = modifier.getRightTrajectory();
	        
	        File myFile_right = new File("C:\\Pathfinder\\mp_right_test_arc.csv");
	        Pathfinder.writeToCSV(myFile_right, right);
	        
	        File myFile_left = new File("C:\\Pathfinder\\mp_left_test_arc.csv");
	        Pathfinder.writeToCSV(myFile_left, left);
	        
	        File myFile_center = new File("C:\\Pathfinder\\autonomous\\mp_x_y_center_test_arc.csv");
	        Pathfinder.writeToCSV(myFile_center, trajectory);
	        
	        //System.out.println(boundHalfDegrees(r2d(6.278141)));
	        
	        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	        // Team 3654 Modification:  create a single combined profile used by both left and right motor, instead of two files
	        StringBuffer sb = new StringBuffer();
	        Trajectory.Segment right_seg;
	        Trajectory.Segment left_seg;
	        
	        sb.append("0,0,0,0,0,0,0,");
	        sb.append(time_step_ms);
	        sb.append("\n");
	        
	        int wait_cycle = (int)(wait_in_ms/ time_step_ms);
	        if( wait_cycle > 0 ) {
	        	for (int w = 0; w < wait_cycle; w++) {
		        	sb.append((w+1));
		        	sb.append(",");
		        	sb.append(0.0);// don't swap left and right position for pigeon
		        	sb.append(",");
		        	sb.append(0.0);
		        	sb.append(",");     
		        	sb.append(0.0);
		        	sb.append(",");
		        	sb.append(0.0);
		        	sb.append(",");
		        	sb.append(0.0);
		        	sb.append(",");
		        	sb.append(0.0);
		        	sb.append(",");
		        	sb.append(time_step_ms);
		        	sb.append("\n");
	        	}
	        }
	        
	        /*  Rounding error causes the first a few row having radian 6.28xx (about 2 pie, 360 degree)
	         *  Must call boundHalfDegree() to correct that
	         *  dt,x,y,position,velocity,acceleration,jerk,heading
				0.020000,0.000000,0.000000,0.000012,0.001200,0.060000,3.000000,6.283157
				0.020000,0.000000,0.000000,0.000060,0.003600,0.120000,3.000000,6.283164
	         * 
	         */
			double turnDegree = 0.00;
			double previousLeftturnDegree = 0.00;
	        double previousRightturnDegree = 0.00;
	        for (int i = 0; i < trajectory.length(); i++) {
				right_seg = right.get(i);
				left_seg = left.get(i);
				// NOTE:  left, right may need swap if the motor output is inverted
			 
				// using meter as unit: linecount, right position, right velocity, left position, left velocity, time_step 
			 
				//2,000196,0.019565,0.000196,0.019565,20
				 //sb.append((i+1));
				
				// WITH Pigeon, no need swap the direction, but Navx is the opposite to the Pathfind's heading's sign
				
				// PATH finder heading:  CCW is positive, CW is negative,  Forward is positive X
				 
				 sb.append((i+wait_cycle+1));
				 sb.append(",");
				 sb.append(String.format("%.3f",  left_seg.position));
				 sb.append(",");
				 sb.append(String.format("%.3f",  left_seg.velocity));
				 sb.append(",");
				 sb.append(String.format("%.3f",  right_seg.position));// don't swap left and right position for Pigeon, do it for Navx
				 sb.append(",");
				 sb.append(String.format("%.3f",  right_seg.velocity));
				 sb.append(",");     
				 
				 //***** IMPORTANT:   heading in pathfinder output is radian, need convert it to degree
				 
				 //****  BUG in software:  beginning and ending heading often WRONG,  need manually correct it !!!!!!!!!!
				 
				 //****  ANOTHER Mis-match with our breadboard:  LEFT and RIGHT are SWITCHed in our config
				 
				 //****  THIRD issue:  pigeon use continuous angle ±23040, jaci's angle output has issue once over t ±180 (jaci's program only good at <180 and >-180 heading range), or even the first non-value can be 358 instead of -2
				 
				 
				 turnDegree = r2d(left_seg.heading);
				 turnDegree = boundHalfDegrees(turnDegree);
				 
				 
				 // Handle the issue of Angle generated by Jaci's algorithm:  when angle cross +- 180, there is a jump in angle about +- 360 degree. 
				 // Encountered in 2021 challenge's barrel path which has three full circles
				 if( Math.abs(turnDegree - previousLeftturnDegree) > MAX_TURN_RATE_IN_20_MS ) {
					 // means Jaci's code has issue with angle, need correct it. Normally it is over +-180, need +- 360 to fix it
					 if( previousLeftturnDegree < -170 && turnDegree > 170  && leftJumpDirection == 0) {
						 //turnDegree = turnDegree - 360;
						 leftJumpDirection = -1;
					 }
					 else if( previousLeftturnDegree > 170 && turnDegree < -170  ) {
						 //turnDegree = turnDegree ; // not doing anything on negative value when from postive to negative jump
						 if(  leftJumpDirection == -1) {
							 leftJumpDirection = 0; // next round will not jump, turn it off
						 }
						 else if( leftJumpDirection == 0) {
							 leftJumpDirection = 1;
						 }
					 }
				 }
								 
				 previousLeftturnDegree = turnDegree;
				 
				 
				 if(leftJumpDirection == -1) {
					 turnDegree = turnDegree - 360;
				 }
				 else if(leftJumpDirection == 1) {
					 turnDegree = turnDegree + 360;
				 }
				 
				 sb.append(String.format("%.2f", turnDegree));
				 sb.append(",");
				 
						 
				 
				 turnDegree = r2d(right_seg.heading);
				 turnDegree = boundHalfDegrees(turnDegree);
				 
				 
				 
				 // Handle the issue of Angle generated by Jaci's algorithm:  when angle cross +- 180, there is a jump in angle about +- 360 degree. 
				 // Encountered in 2021 challenge's barrel path which has three full circles
				 if( Math.abs(turnDegree - previousRightturnDegree) > MAX_TURN_RATE_IN_20_MS ) {
					 // means Jaci's code has issue with angle, need correct it. Normally it is over +-180, need +- 360 to fix it
					 if( previousRightturnDegree < -170 && turnDegree > 170  && rightJumpDirection == 0) {
						 //turnDegree = turnDegree - 360;
						 rightJumpDirection = -1;
					 }
					 else if( previousRightturnDegree > 170 && turnDegree < -170  ) {
						 //turnDegree = turnDegree ; // not doing anything on negative value when from postive to negative jump
						 if(  rightJumpDirection == -1) {
							 rightJumpDirection = 0; // next round will not jump, turn it off
						 }
						 else if( rightJumpDirection == 0) {
							 rightJumpDirection = 1;
						 }
					 }
				 }
				 
				 previousRightturnDegree = turnDegree;

				 if(rightJumpDirection == -1) {
					 turnDegree = turnDegree - 360;
				 }
				 else if(rightJumpDirection == 1) {
					 turnDegree = turnDegree + 360;
				 }
				 
				 sb.append(String.format("%.2f", turnDegree));
				 sb.append(",");
				 
		   
				 
				 sb.append(time_step_ms);
				 if( i < (trajectory.length()  - 1 )) {
					 sb.append("\n");
				 }
			 
			
		 }

	        // Write the String data into a file
	        BufferedWriter bw = null;
			FileWriter fw = null;

			try {
				fw = new FileWriter(FILENAME);
				bw = new BufferedWriter(fw);
				bw.write(sb.toString());
				System.out.println("Done");
			} catch (IOException e) {
				e.printStackTrace();
			} finally {
				try {
					if (bw != null)
						bw.close();
					if (fw != null)
						fw.close();
				} catch (IOException ex) {
					ex.printStackTrace();
				}
			}
	           
			long endtime = System.nanoTime();
	        
			System.out.println("The time used  = "+( endtime - starttime)/1000000 +" ms");

	    	}
	    	catch(Exception e) {
	    		System.out.println(e.getMessage());
	    	}
	   } // main
	
	
	/**
     * Convert radians to degrees. This is included here for static imports.
     * @param radians the radians input
     * @return the input in degrees
     */
    public static double r2d(double radians) {
        return Math.toDegrees(radians);
    }

    /**
     * Bound an angle (in degrees) to -180 to 180 degrees.
	 * @param angle_degrees an input angle in degrees
	 * @return the bounded angle
     */
    public static double boundHalfDegrees(double angle_degrees) {
        while (angle_degrees >= 180.0) angle_degrees -= 360.0;
        while (angle_degrees < -180.0) angle_degrees += 360.0;
        return angle_degrees;
    }

	}