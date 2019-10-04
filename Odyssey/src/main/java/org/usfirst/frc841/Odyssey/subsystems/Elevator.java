// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc841.Odyssey.subsystems;

import org.usfirst.frc841.Odyssey.C;
import org.usfirst.frc841.Odyssey.commands.*;
import org.usfirst.frc841.lib.PID.PIDControlLoop; // Using library to control the sensors and import the constants

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class Elevator extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private DigitalInput upperlimit;
    private DigitalInput lowerlimit;
    private WPI_TalonSRX elevator1;
    private WPI_TalonSRX elevator2;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

	
    // In order to obtain a smooth behavior of the elevator, we apply the PID functions to
    // the elevator as well. We create a new object to do the calculations and we will
    // set up PID after.
	
    // The setup of the PID are in the C file, where all the constants are stored, are the ones 
    public Elevator.CLoop ploop;
    
    double x[] = {1,2,3};  //x and y are not longer used in the function CLoop but can be use them in the future.
    double y[] = {0,0,0};  //x and y help to setup the settings to compensate gravity and other forces.
    private double period = 0.1;
    private double zero;
    //constructor

    public Elevator() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        upperlimit = new DigitalInput(3);
        addChild("upperlimit",upperlimit);
        
        
        lowerlimit = new DigitalInput(6);
        addChild("lowerlimit",lowerlimit);
        
        
        elevator1 = new WPI_TalonSRX(11);
        
        
        
        elevator2 = new WPI_TalonSRX(12);
        
        
        

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    
    ploop = new Elevator.CLoop(this, this.x,this.y, (long) Math.abs(this.period*100));
    ploop.SetOutputLimits(C.e_limitMinus, C.e_limitPlus);
    ploop.setTunings(C.e_p, C.e_i, C.e_d);
    ploop.updateSetpoint(4807);
    //this.zero = elbow.getSensorCollection().getPulseWidthPosition();
    //this.zero = 2850- this.zero;

    }

    public void elevatorUp() {
        this.elevator1.set(1);
        this.elevator2.set(1);
    }

    public void elevatorDown() {
        if (this.lowerlimit.get())
        {
            this.elevator1.set(-1);
            this.elevator2.set(-1);
        }
        else
        {
            this.elevator1.set(0);
            this.elevator2.set(0);
        }
    }

    public void elevatorStop() {
        this.elevator1.set(0);
        this.elevator2.set(0);
    }

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

    }

    public void setElevatorMotorSpeed(double speed){
        elevator1.set(-speed);
        elevator2.set(-speed);  //TODO: Check the polarity of the motors.
    }

    public double getElevatorSensor(){
        return elevator1.getSensorCollection().getQuadraturePosition();
    }

    public boolean getBottomLimitSensor(){
        return this.lowerlimit.get();
    }

    public void resetBottomLimitSensor(){
        this.elevator1.getSensorCollection().setQuadraturePosition(0, 0); // set up the value of the sensor to 0
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    /* This Class links the Intakes Control loop to the subsystem
	 *
	 */
    public class CLoop extends PIDControlLoop {
		Elevator subsystem;

		public CLoop(Elevator subsystem, double[] X, double[] Y, long SampleTime) {
			super(X, Y, SampleTime);
			this.subsystem = subsystem;
			// TODO Auto-generated constructor stub
		}

		@Override
		public void setOutput(double value) {

			subsystem.setElevatorMotorSpeed(value);
			
			// System.out.println("Output: " + value);
		}

		@Override
		public double getSensorReading() {
           
			// System.out.println("in: " + subsystem.getUpperWheelSpeed());
			return  subsystem.getElevatorSensor();//subsystem.getAngle();
		}
		@Override
		public void update() {
		//	SmartDashboard.putString("DB/String 2",
		//			"Angle: " + Math.floor(this.subsystem.getAngle() * 100 / 100.0));
            SmartDashboard.putNumber ("Height",this.subsystem.getElevatorSensor());
            
            //Reset the reading of the motor sensor when the botton sensor
            if (this.subsystem.getBottomLimitSensor()) //TODO: Check the polarity.
            {
                this.subsystem.resetBottomLimitSensor();
            }

		}
	}

}

