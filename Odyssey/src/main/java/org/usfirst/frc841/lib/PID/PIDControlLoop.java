package org.usfirst.frc841.lib.PID;

import java.util.Timer;
import java.util.TimerTask;
/**
 * Team 841 version of the PID loop with Anti-Integral Wind-up, Anti-Derivative kick,
 * and feed-forward implementation 
 * @author Team841-P01
 *
 */
public class PIDControlLoop {

	// PID variables
	private double Output = 0;
	private double Setpoint = 0;
	private double ITerm, lastInput = 0;
	private double kp, ki, kd = 0;
	private double SampleTime = 1000;
	private double outMin, outMax = 0;

	// PID output reversal
	static private boolean DIRECT = false;
	static private boolean REVERSE = false;

	private boolean controllerDirection = DIRECT;

	// PID implementations variables
	private boolean isDestinationReached = false;
	private boolean enablePID = false;

	// feed-forward variables;
	private int ffpointer = 0;
	private double ffTableY[];
	private double ffTableX[];
	private double slope[];
	private double b[];

	// Moving average object
	SMA averageError;

	private Timer timer;

	public PIDControlLoop(double X[], double Y[], long SampleTime) {
		// initialize the tables
		ffTableY = new double[Y.clone().length];
		ffTableX = new double[X.clone().length];
		ffTableY = Y.clone();
		ffTableX = X.clone();
		slope = new double[ffTableY.length];
		b = new double[ffTableY.length];

		// Calculate Y = mX + b equations
		for (int i = 0; i < (ffTableY.length - 1); i++) {
			slope[i] = (ffTableY[i] - ffTableY[(i + 1)]) / (ffTableX[i] - ffTableX[(i + 1)]);
			b[i] = ffTableY[i] - slope[i] * ffTableX[i];
		}

		// Initialize Moving average of 15
		averageError = new SMA(15);

		// SetsSampleTime
		this.SampleTime = SampleTime;

		timer = new Timer();
		timer.schedule(new Update(this), 0L, SampleTime);

	}

	public PIDControlLoop(double X[], double Y[], long SampleTime, double Tunings[]) {

		PIDControlLoop mynew = new PIDControlLoop(X, Y, SampleTime);

		if (Tunings == null) {
			mynew.setTunings(.0, .0, .0);
		} else {
			mynew.setTunings(Tunings[0], Tunings[1], Tunings[2]);
		}
	}
	/**
	 * Class that implements the periodic task for PID Loop
	 *
	 */
	class Update extends TimerTask {
		private PIDControlLoop c;

		public Update(PIDControlLoop pidLoop) {
			this.c = pidLoop;
		}

		@Override
		public void run() {
			// This runs the PID loop.
			c.update();

			// This reads the sensor input, Computes the PID value and sets the
			// output
			if (enablePID) {
				c.setOutput(c.Compute(c.getSensorReading()));
			} else {
				//If disabled then clear PID variables for a fresh start
				lastInput = c.getSensorReading();
				ITerm = 0;
				averageError.clear();
			}

		}
	}

	/**
	 * This calculates the feed forward value from the feed forward table
	 * @param input
	 * @return feed forward for PID computation
	 */
	public double getFFValue(double input) {
		// find current ff data;
		double data = Math.ceil(input);

		// find current ff data;
		for (int i = 0; i < (slope.length - 1); i++) {
			if (data > ffTableX[i]) {
				ffpointer = i;
			}
		}
		return (slope[ffpointer] * input + b[ffpointer]);
	}

	/**
	 *  Sets PID tuning variables
	 * @param Kp
	 * @param Ki
	 * @param Kd
	 */
	public void setTunings(double Kp, double Ki, double Kd) {
		if (Kp > 0 || Ki > 0 || Kd > 0) {
			double SampleTimeInSec = SampleTime / 1000;

			kp = Kp;
			ki = Ki * SampleTimeInSec;
			kd = Kd / SampleTimeInSec;
			if (controllerDirection == REVERSE) {
				kp = (0 - kp);
				ki = (0 - ki);
				kd = (0 - kd);

			}
		}
	}

	/**
	 * Set sample time in milliSeconds
	 * @param NewSampleTime
	 */
	public void setSampleTime(double NewSampleTime) {
		if (NewSampleTime > 0) {
			double ratio = NewSampleTime / SampleTime;

			ki *= ratio;
			kd /= ratio;

			SampleTime = NewSampleTime;
			// System.out.println(ki);
		}
	}

	/**
	 * Sets the limit for the PID output
	 * @param Min
	 * @param Max
	 */
	public void SetOutputLimits(double Min, double Max) {
		if (Min < Max) {
			outMin = Min;
			outMax = Max;

			if (Output > outMax) {
				Output = outMax;
			} else if (Output < outMin) {
				Output = outMin;
			}
			if (ITerm > outMax) {
				ITerm = outMax;
			} else if (ITerm < outMin) {
				ITerm = outMin;
			}
		}
	}

	/*
	 * //Clears up all data from past PID loop public void initialize(double
	 * input){ lastInput = input; ITerm = Output; if (ITerm > outMax){ ITerm =
	 * outMax; } else if(ITerm < outMin){ ITerm = outMin; } }
	 */

	/**
	 * Clears the PID variables for a fresh start
	 */
	public void initialize() {
		lastInput = 0;
		ITerm = 0;
		Output = 0;

	}

	/**
	 * Set The PID output polarity for a reverse or forward direction
	 * @param Direction
	 */
	public void SetControllerDirection(boolean Direction) {
		controllerDirection = Direction;
	}

	/**
	 * Computes the PID values to be updated
	 * @param input
	 * @return returns the PID output value
	 */
	public double Compute(double input) {

		//calculate error
		double error = Setpoint - input;
		
		// update average
		averageError.compute(error);
		
		//Calculate and update Iterm and limit Iterm to prevent integral wind-up
		ITerm += (ki * error);
		if (ITerm > outMax) {
			ITerm = outMax;
		} else if (ITerm < outMin) {
			ITerm = outMin;
		}
		//Calculate d term using samples only to prevent derivative kick
		double dInput = (input - lastInput);

		// Compute PID Output
		Output = kp * error + ITerm - kd * dInput + this.getFFValue(Setpoint);

		//Limit output if too big
		if (Output > outMax) {
			Output = outMax;
		} else if (Output < outMin) {
			Output = outMin;
		}

		// Remember some variables for next time
		lastInput = input;

		//Gets average limit to see if PID is within range of error tolerance
		if (Math.abs(averageError.currentAverage()) < Math.abs(Setpoint * 0.05)) {
			isDestinationReached = true;
		} else {
			isDestinationReached = false;
		}
		// Return calculated values.
		return Output;

	}

	/**
	 * This method allows us to know when to disable the PID Loop or go to the
	 * next step
	 * 
	 * @return is Controller within goal?
	 */
	public boolean isDestinationReached() {
		return isDestinationReached;
	}

	/**
	 * This updates the goal for the PID loop to follow.
	 * 
	 * @param ref
	 */
	public void updateSetpoint(double ref) {
		Setpoint = ref;
	}

	/**
	 * This Enables the PID loop
	 */
	public void enablePID() {
		enablePID = true;
		isDestinationReached = false;
	}

	/**
	 * This disables the PID loop
	 */
	public void disablePID() {
		enablePID = false;
	}

	/**
	 * Returns state of PID loop
	 * 
	 * @return
	 */
	public boolean isPIDEnabled() {
		return enablePID;
	}

	/**
	 * This output method needs to be defined be the subsystem class for it to do anything
	 * @param value
	 */
	public void setOutput(double value) {
		System.out.println("Need to override the SetOutput Methode of PIDLoop to Work");

	}

	/**
	 * This input method needs to be defined be the subsystem class for it to do anything. It is meant
	 * to be overridden
	 * @return sensor reading
	 */
	public double getSensorReading() {
		System.out.println("Need to override the getSensorReading of PIDLoop to Work");
		return 0;
	}

	/**
	 * This method is meant to be overridden
	 */
	public void update() {

	}

}
