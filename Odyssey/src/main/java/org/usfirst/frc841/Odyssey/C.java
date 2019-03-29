package org.usfirst.frc841.Odyssey;
/**
 * In this class we are storing all the constants and tunings used in all the commands and subsystems
 * @author Team841-P01
 *
 */
public class C {

	private C(){
		// Prevents instantiation
	}

	// STRUCTURE OF THE CONSTANTS CONTAINED IN THIS DOCUMENT
	//public static final double test = 10.001;	
	
	// CONSTANTS FOR THE DRIVETRAIN
	public static final double throttleDeadband = 0.02; 
	public static final double wheelDeadband = 0.02;	
	public static final double sensitivityHigh = 0.85;	
	public static final double sensitivityLow = 0.75;
	public static final double centervalue = 140;
	public static final double tolerance = 10;
	public static final int currentlimit = 40;
	
	// AUTONOMOUS CONSTANTS
	public static final double autoDrivingDistance = .5; // In time, is seconds driving from the starting point to one of the nodes of movement in the autonomous mode.
	public static final double autoTurningDistance = .4; // In time, is seconds to turn until 45 degrees from a 0 turning speed.	
	public static final double autoMotorSpeed = .5;
	public static final double autoMotorLowSpeed = .3;
	//public static int nAutonomousMode = 0;
	
	
	//Hatch Elbow Constatns;
	public static final double h_p =0.0009;
	public static final double h_i = 0;
	public static final double h_d =0.00022;

	//Elevator Constants
	public static final double e_p = 0;
	public static final double e_i = 0;
	public static final double e_d = 0;
	public static final double e_limitMinus = -.4;
	public static final double e_limitPlus = .4;
}