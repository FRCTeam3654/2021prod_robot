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


public class Tank {

	
    public static void main(String[] args) {
    	
    	
       String FILENAME = "C:\\Pathfinder\\mp_20ms_in_meter.csv";
    	
    	try {
    	
    	long starttime = System.nanoTime();
    	
    	int time_step_ms = 20;
    	
    	double wheelbaseinmeter = 0.7;// 0.635;//0.7;// 0.8;
    	
    	
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
       Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, time_step, 1.2, 1.2, 30.0); 
       
    	//Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, time_step, 0.6, 0.7, 20.0); 
       
    	//Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, time_step, 1.2, 1.5, 40.0); 
       
        Waypoint[] points = new Waypoint[] {
        		    new Waypoint(0, 0, 0),  
        		    
        		
        		    
        		  //  new Waypoint(3.2, 0.0, 0)  //CrossTheLine
        		  //  new Waypoint(2.0, 0.7, Pathfinder.d2r(60)) // test turning 60 degree 
        		    
        		    new Waypoint(1.5, 0.0, 0) // test forward 1.5 m
        		    
        		 //   new Waypoint(2.44, -1.37, 0)  // test short ML 
        		    
        		  //---------------------------------------------------------------------------------------------------- 
        		    
        		 // this is : center to switch  
          		  // FRC 2018 case#1: 96 inch forward, 54 inch side way (140 - 32- 2x6 = 96, 6 x12 - 3 x 12 /2 = 54) 
              	// new Waypoint(2.44, 1.37, 0)  //MRSwitch, MRScale --- this is : center to switch
        		 // new Waypoint(2.44, -1.37, 0)  //MLSwitch, MLScale --- this is : center to switch   
          		    
        		  //---------------------------------------------------------------------------------------------------- 
          		    
        		    
        		 // this is : side to switch, need middle point 2.5 to ensure straight
          		    // FRC 2018 case#2: 124 inch forward, 43 inch side way (if use 3 inches bump, it is 43 inches not 40) (Dories's # is 122x40)
        		  
        		   //   new Waypoint(2.5, 0.0, 0),  
        		   // new Waypoint(3.17, 1.092, Pathfinder.d2r(90))  //LLSwitch -- this is : side to switch, need middle point 2.5 to ensure straight
        		  // new Waypoint(3.17, -1.092, Pathfinder.d2r(90))  //RRSwitch
        		  //---------------------------------------------------------------------------------------------------- 
          		      
        		  
        		 // this is : side to scale via platform with no turning
          		 // FRC 2018 case#3:  255.65 inch forward, 40 (or 34) inch side way  , add a middle point to avoid touching switch
        		    
        		     // new Waypoint(2.5, 0.0, 0),  
                	 // new Waypoint(6.49, 1.016, 0)  //LLScaleNoTurn --- this is : side to scale
                	 // new Waypoint(6.49, -1.016, 0)  //RRScaleNoTurn --- this is : side to scale
        		 
                	  
        		  //---------------------------------------------------------------------------------------------------- 
          		  
        		 // this is : side to scale straight with 90 turn
              	  // FRC 2018 case#4: 279.65 inch forward, 22 inch side way  (should 279.65 x 22 inches)
        		   
        		   // new Waypoint(6.3, 0.0, 0),  // setting for 22 inches
              	  // new Waypoint(7.10, 0.56, Pathfinder.d2r(90))  //LLScaleTurn-- this is : side to scale
              	  // new Waypoint(7.10, -0.56, Pathfinder.d2r(90))  //RRScaleTurn-- this is : side to scale
              	 
        		    //new Waypoint(6.5, 0.0, 0),  // setting for 16 inches
        		    //new Waypoint(7.10, 0.40, Pathfinder.d2r(90)) //LLScaleTurn-- this is : side to scale --- 16 inches 
        		    //new Waypoint(7.10, -0.40, Pathfinder.d2r(90))  //RRScaleTurn-- this is : side to scale --- 16 inches 
        		    
    //    		    new Waypoint(6.7, 0.0, 0),  // setting for 10 inches
   //     		    new Waypoint(7.10, 0.254, Pathfinder.d2r(90)) //LLScaleTurn-- this is : side to scale --- 10 inches 
        		   // new Waypoint(7.10, -0.254, Pathfinder.d2r(90))  //RRScaleTurn-- this is : side to scale --- 10 inches 
        		  //---------------------------------------------------------------------------------------------------- 
            		  
              	
        		  // FRC 2018  case #5 cross the field in middle to scale: 261 to platform, 39x34 robot, platform width 104
              	  // dimension: vertidal 261 - 34/2 - 39/2 = 224 .   horizontal 104 + 40+6 = 150 inches t
        		   
        		   // new Waypoint(3.0, 0.0, 0),
        		   // new Waypoint(5.69, 3.81, Pathfinder.d2r(90)) //LRSwitch,LRScale  ---3.0 middle point to ensure it will touch switch 
        		   // new Waypoint(5.69, -3.81, Pathfinder.d2r(90)) //RLSwitch , RLScale  ---3.0 middle point to ensure it will touch switch 
        		    
        		    
        		    
        		    
        		    
        		  //---------------------------------------------------------------------------------------------------- 
 
              	 // testing cross the field at the bottom
          		    /*
          		    new Waypoint(1.27, 1.02, Pathfinder.d2r(90)),
          		    new Waypoint(0.254, 4.37, Pathfinder.d2r(90)),
          		    new Waypoint(2.54, 6.15, Pathfinder.d2r(0)),
          		    
          		    // case #4
          		    new Waypoint(6, 6.2, Pathfinder.d2r(0)),  // this one is needed to ensure it is in range
          		    new Waypoint(7.09, 5.13, Pathfinder.d2r(-90))
          		    
          		    // case #3 
          		    //new Waypoint(6.49, 4.67, 0)  // this is case#3 : other side to scale
              	  
              	  */ 
          		    
          		   /*
          		 ///-------- test cross the field at the middle
          		    new Waypoint(4.00, 0, Pathfinder.d2r(0)),
          		    new Waypoint(5.52, 1.52, Pathfinder.d2r(90)),
          		    new Waypoint(5.52, 4.49, Pathfinder.d2r(90)),
          		    new Waypoint(5.84, 5.13, Pathfinder.d2r(0))
          		 */
          		    
          		    
          		  /// ------ test sharp turn at bottom
          		    //new Waypoint(0.3, 0.4, Pathfinder.d2r(50)),
          		    //new Waypoint(0.4, 1.02, Pathfinder.d2r(90))
          		    /*
          		    new Waypoint(0.4, 4.37, Pathfinder.d2r(90)),
          		    new Waypoint(2.54, 6.15, Pathfinder.d2r(0)),
          		    new Waypoint(6, 6.2, Pathfinder.d2r(0)),  
          		    new Waypoint(7.09, 5.13, Pathfinder.d2r(-90))
          		    */
          		    
          		    /*
          		  ///-------- test cross the field at the middle -- small test
          		    new Waypoint(1.00, 0, Pathfinder.d2r(0)),
          		    new Waypoint(2.0, 1.52, Pathfinder.d2r(90)),
          		    new Waypoint(2.0, 2.0, Pathfinder.d2r(75)),
          		    new Waypoint(2.3, 2.13, Pathfinder.d2r(0))  
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
        
        File myFile_right = new File("C:\\Pathfinder\\mp_right_test.csv");
        Pathfinder.writeToCSV(myFile_right, right);
        
        File myFile_left = new File("C:\\Pathfinder\\mp_left_test.csv");
        Pathfinder.writeToCSV(myFile_left, left);
        
        File myFile_center = new File("C:\\Pathfinder\\mp_center_test.csv");
        Pathfinder.writeToCSV(myFile_center, trajectory);
        
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Team 3654 Modification:  create a single combined profile used by both left and right motor, instead of two files
        StringBuffer sb = new StringBuffer();
        Trajectory.Segment right_seg;
        Trajectory.Segment left_seg;
        
        sb.append("1,0,0,0,0,");
        sb.append(time_step_ms);
        sb.append("\n");
        
        int wait_cycle = (int)(wait_in_ms/ time_step_ms);
        if( wait_cycle > 0 ) {
        	for (int w = 0; w < wait_cycle; w++) {
	        	sb.append((w+1));
	        	sb.append(",");
	        	sb.append(0.0);// swap left and right position
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
        
        
        for (int i = 0; i < trajectory.length(); i++) {
               right_seg = right.get(i);
               left_seg = left.get(i);
               // NOTE:  left, right may need swap if the motor output is inverted
            
               // using meter as unit: linecount, right position, right velocity, left position, left velocity, time_step 
            
               //2,000196,0.019565,0.000196,0.019565,20
            	//sb.append((i+1));
            	
            	sb.append((i+wait_cycle+1));
            	sb.append(",");
            	sb.append(String.format("%.5f", right_seg.position));// swap left and right position
            	sb.append(",");
            	sb.append(String.format("%.5f", right_seg.velocity));
            	sb.append(",");     
            	sb.append(String.format("%.5f", left_seg.position));
            	sb.append(",");
            	sb.append(String.format("%.5f", left_seg.velocity));
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
    }

}