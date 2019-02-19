package org.usfirst.frc841.lib;

public class PIDLoop {



	//PID variables  
	private long lastTime;
	private double Input, Output, Setpoint;
	private double ITerm, lastInput;
	private double kp, ki, kd;
	private double SampleTime = 1000;
	private double outMin, outMax;
	private boolean inAuto = false;

	//PID Manual info
	private boolean MANUAL = false;
	private boolean AUTOMATIC = true;
	 
	//PID output reversal
	private boolean DIRECT = false;
	private boolean REVERSE = true;
	 
	private boolean controllerDirection = DIRECT;

	 
	//feed-forward variables; 
	private int ffpointer = 0;
	private double ffTableY[]; 
	private double ffTableX[];
	private double slope[];
	private double b[];
	 

	public PIDLoop(double X[] , double Y[]){
		//initialize the tables
		ffTableY = new double[Y.clone().length];
		ffTableX = new double[X.clone().length];
		ffTableY = Y.clone();
		ffTableX = X.clone();
		slope = new double[ffTableY.length];
		b = new double[ffTableY.length];
	  
	     //Calculate Y = mX + b equations
		for(int i=0;i<(ffTableY.length-1);i++){
			slope[i] = (ffTableY[i] - ffTableY[(i+1)]) / 
			(ffTableX[i] - ffTableX[(i+1)]);
			b[i] = ffTableY[i] - slope[i]*ffTableX[i];
	    }

	}
	
	//returns current ff value. 
	public double getFFValue(double input){
		//find current ff data;
		double data =Math.ceil(input);
	    
	      //find current ff data;
		for (int i=0; i < (slope.length-1);i++){ 
			if(data > ffTableX[i]){
				ffpointer = i;
			}
		}
	   return (slope[ffpointer]*input+b[ffpointer]);
	 }
	
	// Sets positive PID values 
	public void SetTunings(double Kp, double Ki, double Kd){
		if(Kp > 0 || Ki > 0 || Kd > 0 ){
			double SampleTimeInSec = SampleTime/1000;
	   
			kp = Kp;
			ki = Ki * SampleTimeInSec;
			kd = Kd / SampleTimeInSec;
			if(controllerDirection == REVERSE){
				kp = (0 - kp);
				ki = (0 - ki);
				kd = (0 - kd);
	     
			}
		}
	}
	
	// Sets Sample Time in milliseconds
	public void SetSampleTime( double NewSampleTime){
		if (NewSampleTime > 0){
			double ratio = NewSampleTime / SampleTime;
	   
			ki *=ratio;
			kd /=ratio;
	   
			SampleTime = NewSampleTime;
			//System.out.println(ki);
		}
	}
	
	// Sets Limit to Output
	public void SetOutputLimits( double Min, double Max){
		if(Min < Max){
			outMin = Min;
			outMax = Max;
	   
			if(Output > outMax){
				Output = outMax;
			}
			else if(Output < outMin){
				Output = outMin;
			}
			if (ITerm > outMax){
				ITerm = outMax;
			}
			else if(ITerm < outMin){
				ITerm = outMin;
			}
		}
	}
	
	 // Turns on PID loop
	// public void SetMode( boolean Mode){
	 // boolean newAuto = Mode;
	 // if(newAuto && !inAuto){
	 //  Initialize();
	 // }
	 // inAuto =  newAuto;
	 //}
	
	//Clears up all data from past PID loop
	public void Initialize(double input){
		lastInput = input;
		ITerm = Output;
		if (ITerm > outMax){
			ITerm = outMax;
		}
		else if(ITerm < outMin){
			ITerm = outMin;
		}
	 }
	
	//Set the PID output polarity
	public void SetControllerDirection (boolean Direction){
		controllerDirection = Direction;
	}
	 
	//Computes the PID values to be updated
	 public double Compute(double input){
	  
	    double error = Setpoint - input;
	    ITerm += (ki * error);
	    if( ITerm > outMax ){
	     ITerm = outMax;
	    }
	    else if( ITerm < outMin ){
	     ITerm = outMin;
	    }
	    double dInput = (input - lastInput);
	    
	    //Compute PID Output
	    Output = kp * error + ITerm - kd * dInput + this.getFFValue(Setpoint);
	    
	    if( Output > outMax ){
	     Output = outMax;
	    }
	    else if( Output < outMin ){
	     Output = outMin;
	    }
	    
	    //Remember some variables for next time
	    lastInput = input;
	  //  lastTime = now;
	    return Output;
	   }

	 
	 // Update Setpoint
	 public void SetReference(double ref){
	  Setpoint =  ref;
	 }

	 
}
