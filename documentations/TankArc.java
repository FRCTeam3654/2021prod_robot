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
	    	
	    	double wheelbaseinmeter = 0.5;// 0.635;//0.7;// 0.8;
	    	
	    	
	    	//boolean useRotationAsUnit = false;
	    	//boolean useRotationAsUnit =  true;
	    	//double effective_wheel_diameter_in_inches = 5.645;//inches
	    	//double meter2RotationConversion = 100 / 3.14 / effective_wheel_diameter_in_inches / 2.54;
	    	double time_step = ((double)time_step_ms )/ 1000; // second,  = 5 ms
	    	
	    	//int wait_in_ms = 3000;
	    	
	    	int wait_in_ms = 0;
	    	
	    	
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
	       
	    	
	    	Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, time_step, 1.7, 2.0, 60.); 
		       
	    	
	    	
	    	
	    	/*   Positive Y is on the left side
	    	 *   Positive X is the straight forward
	    	 *   Positive heading is Counter-clock-wise
	    	 * 
	    	 * 
	    	 */
	    	
	    	
	    	// nowing that Pathfinder paths shouldn’t go negative,
	    	
	    	// 2020 robot :  left side Y is positive
	    	
	    	
	    	//2021:   Glantic Search A:  tan(71.565)=3  (0,0,0) -> (1.524, 0.0, 0),  (0,0, Pathfinder.d2r(30) ) --> (1.524, -0.762, Pathfinder.d2r(30)),  (0,0, Pathfinder.d2r(30) ) ->(0.762, 2.286, Pathfinder.d2r(30)) 
	    	
	    	//        Barrel Path:  new Waypoint( 2.286, 0, Pathfinder.d2r(0) ), new Waypoint(2.99, -0.762, Pathfinder.d2r(-90)) , new Waypoint(2.286, -1.524, Pathfinder.d2r(-180)),new Waypoint(1.524, -0.762, Pathfinder.d2r(-270)) , new Waypoint(2.286, 0, Pathfinder.d2r(-360))
	    	
	        Waypoint[] points = new Waypoint[] {
	        		    //new Waypoint(0, 0, 0),
	        		
	        		    // working Barrel Path:
	        		    new Waypoint(0, 0, 0),
	        		    new Waypoint( 2.286, 0, Pathfinder.d2r(0) ),
	        		    new Waypoint(2.99, -0.762, Pathfinder.d2r(-90)) ,
	        		    new Waypoint(2.286, -1.524, Pathfinder.d2r(-180)),
	        		    new Waypoint(1.524, -0.762, Pathfinder.d2r(-270)) ,
	        		    new Waypoint(2.286, 0, Pathfinder.d2r(-360)),
	        		    new Waypoint(4.572, 0, Pathfinder.d2r(0)),
	        		    new Waypoint(5.334, 0.762, Pathfinder.d2r(90)),
	        		    new Waypoint(4.572, 1.524, Pathfinder.d2r(180)),
	        		    new Waypoint(3.81, 0.762, Pathfinder.d2r(270)),
	        		    new Waypoint(6.092, -1.524, Pathfinder.d2r(360)),
	        		    new Waypoint(6.854, -0.762, Pathfinder.d2r(90)) ,
	        		    new Waypoint(6.092, 0, Pathfinder.d2r(180))
	        		    // end of working Barrel Path:
	        		    
	        		    
	        		    //new Waypoint(0, 0, Pathfinder.d2r(0))  ,
	        		    //new Waypoint(0.762, -0.762, Pathfinder.d2r(-90)) ,
	        		    //new Waypoint(0, -1.524, Pathfinder.d2r(-180)),
	        		    //new Waypoint(-0.762, -0.762, Pathfinder.d2r(-270)) ,
	        		    //new Waypoint(0, 0, Pathfinder.d2r(-360))  
	        		    
	        
	        		    //new Waypoint(1.5, 0.0, 0)
	        		    
	        		    //Glantic Search A:(90,90),(150,60),(180,150)  in inches 60'' = 152.4, 30 =>76.2, 90=>2.286
	        		    // one non-stop path vs 3 different paths
	        		    //new Waypoint(1.524, 0.0, 0)
	        		    //new Waypoint(3.048, -0.762, 0),
	        		    //new Waypoint(3.81,1.524, 0)
	        		    
	        		   // new Waypoint(1.524, -0.762, Pathfinder.d2r(30))   // second leg of Glantic Search A
	        		    
	        		   // new Waypoint(0.762, 2.286, 0)   // third leg of Glantic Search A
	        		    
	        		
	        		   // new Waypoint(2.52, 1.7 , Pathfinder.d2r(0)) ,
	        		    
	        		    
	        		    //new Waypoint(2.52, -2.3 , Pathfinder.d2r(0)) ,
	        		    //new Waypoint(5.3, -2.3 , Pathfinder.d2r(0))
	        		    
	        		    
	        		    // case #5?
	        		    //new Waypoint(3.3, 0 , Pathfinder.d2r(0)) ,
	        		    //new Waypoint(5.8, 1.73 , Pathfinder.d2r(0))
	        		    
	        		    
	        		    
	        		// new Waypoint(-4, -1, Pathfinder.d2r(-45)),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
	        		//    new Waypoint(-2, -2, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
	        		//    new Waypoint(0, 0, 0)   
	        		
	        		
	        		    /*
	        		     * Front Cargo/Panel:
	        		     *  Front bumper need move vertically 176'' or 4.47 m, leaving 8'' to adjust
	        		     *                			horizontally, need move 31'' wihich is 0.79 m
	        		     * 
	        		     *  Path planning strategy:  
	        		     *  1) move straight to the segment that do S curve movement, say 1 meter for testing
	        		     *  2) use next or last 70'' to do the S curve
	        		     */
	        		  
	        		    
	        		    // **** test with short straight line movement, followed by s-curve in last 70''
	        		    
	        		    
	        		    // Front Panel, Turn RIGHT ( left starting position )
	        		    /*
	        		    new Waypoint(1, 0, 0), 
	        		    new Waypoint(2.78, -0.79, 0), // front Cargo  test   2.78-1 = 1.78 m which is 70''
	        		    */
	        		    
	        		    // Front Panel, Turn LEFT ( right starting position )
	        		    
	        		    //new Waypoint(1, 0, 0), 
	        		    //new Waypoint(2.78, 0.79, 0) // front Cargo  test   2.78-1 = 1.78 m which is 70''
	        		    
	        		    
	        		    
	        		    
	        		   // new Waypoint(2.28, 0.79, 0) ,
	        		   // new Waypoint(2.78, 0.79, 0)
	        		    
	        		    
	        		    // ****  REAL FRONT PANEL, move straight 2.69 m (106'') , followed by 1.78m/70'' s-curve,  TOTAL 176'' or 4.47 m
	        		    /*
	        		    new Waypoint(2.69, 0, 0), 
	        		    new Waypoint(4.47, 0.79, 0) // front Cargo
	        		    */
	        		    
	        		
	        		    /*
	        		     *   Side Panel
	        		     *   Need move vertically 176 +8 + 40 = 224'' + mid of robot (not front bumper for side panel) 12'' =  236    or 6.0 m
	        		     *             horizontally, need give robot some room to adjust, say 24'' or 0.61m
	        		     *             
	        		     *     so straight line can have extra 20'' to straight out itself, 106+20=126'' or 3.2m
	        		     *     the ending s-curve has extra 40'' ( 8+40 +12 - 20 = 40), so 70 + 40 = 110 or 2.79 m
	        		     *   
	        		    new Waypoint(3.2, 0, 0), 
	        		    new Waypoint(6.0, 0.61, 0) // side Panel
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
	            	
	            	
	            	
	            	
	            	turnDegree = r2d(left_seg.heading);
	            	turnDegree = boundHalfDegrees(turnDegree);
	            	sb.append(String.format("%.2f", turnDegree));
	            	sb.append(",");
	            	
	            	turnDegree = r2d(right_seg.heading);
	            	turnDegree = boundHalfDegrees(turnDegree);
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
