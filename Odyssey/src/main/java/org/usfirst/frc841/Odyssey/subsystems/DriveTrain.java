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

import org.usfirst.frc841.Odyssey.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import org.usfirst.frc841.Odyssey.C;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class DriveTrain extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private WPI_TalonSRX left1;
    private WPI_VictorSPX left2;
    private WPI_VictorSPX left3;
    private WPI_TalonSRX right1;
    private WPI_VictorSPX right2;
    private WPI_VictorSPX right3;
    private VictorSP leftLed;
    private VictorSP rightLed;
    private Compressor compressor1;

    private double x = 0;
    private double v = 0;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public DriveTrain() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        left1 = new WPI_TalonSRX(4);
        
        
        
        left2 = new WPI_VictorSPX(5);
        
        
        
        left3 = new WPI_VictorSPX(6);
        
        
        
        right1 = new WPI_TalonSRX(1);
        
        
        
        right2 = new WPI_VictorSPX(2);
        
        
        
        right3 = new WPI_VictorSPX(3);
        
        
        
        leftLed = new VictorSP(0);
        addChild("leftLed",leftLed);
        leftLed.setInverted(false);
        
        rightLed = new VictorSP(1);
        addChild("rightLed",rightLed);
        rightLed.setInverted(false);
        
        compressor1 = new Compressor(0);
        addChild("Compressor 1",compressor1);
        
        

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    left1.configPeakCurrentLimit(20,0);
    right1.configPeakCurrentLimit(20, 0);
    right2.follow(right1);
    right3.follow(right1);
    left2.follow(left1);
    left3.follow(left1);
   


//    NetworkTable limeTable = NetworkTableInstance.getDefault().getTable("limelight");
//    NetworkTable smartTable = NetworkTableInstance.getDefault().getTable("Smart Dashboard");

    }

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new Drive());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
    public double getX(){
        return x;
    }
    public double getV(){
        return v;
    }
    @Override
    public void periodic() {
	   
	    
	    // We create the object to store the table here
	    
    NetworkTable table; 
    NetworkTableEntry tx;
    //NetworkTableEntry ty; 
   // NetworkTableEntry ta; 
    NetworkTableEntry tv; 
	    
        // Update values of the table
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        tv = table.getEntry("tv");

    //    NetworkTableEntry ty = table.getEntry("ty");
      //  NetworkTableEntry ta = table.getEntry("ta");
               
         // read  values periodically
        this.x = tx.getDouble(0.0);
       // this.y = ty.getDouble(0.0);
       // this.area = ta.getDouble(0.0);
        this.v = tv.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        //SmartDashboard.putNumber("LimelightY", y);
        //SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("LimelightVision", v);
        // Note quickturn and shift is taken care of with buttons in OI.
        // Put code here to be run every loop

    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    boolean isHighGear = false;
    private double oldWheel = 0.0;
    private double quickStopAccumulator = 0;
    private boolean isQuickTurn = false;



    /**
     * Enable quick turn AKA classical Arcade drive
     */
    public void setQuickTurn() {
        isQuickTurn = true;
    }

    /**
     * Disable quick turn
     */
    public void resetQuickTurn() {
        isQuickTurn = false;
    }

    /**
     * This method does the Halo drive and is the slim down version of the cheesy
     * poofs drive style. It has the capability to auto shift if uncommented.
     * 
     * @param stick
     */
    public void cheesyDrive(Joystick stick) {


        double wheelNonLinearity;
        double wheel = handleDeadband(getWheel(stick), C.wheelDeadband); // double
                                                                         // wheel
                                                                         // =
                                                                         // handleDeadband(controlBoard.rightStick.getX(),
                                                                         // wheelDeadband);
        double throttle = -handleDeadband(getThrottle(stick), C.throttleDeadband);
        double negInertia = wheel - oldWheel;
        /*
         * if(getAverageSpeed()> 2000){ SetHighGear(); } else if (getAverageSpeed() <
         * 1500){ SetLowGear(); }
         */

        oldWheel = wheel;
        if (isHighGear) {
            wheelNonLinearity = 0.6;
            // Apply a sin function that's scaled to make it feel better.
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / Math.sin(Math.PI / 2.0 * wheelNonLinearity);
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / Math.sin(Math.PI / 2.0 * wheelNonLinearity);
        } else {
            wheelNonLinearity = 0.5;
            // Apply a sin function that's scaled to make it feel better.
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / Math.sin(Math.PI / 2.0 * wheelNonLinearity);
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / Math.sin(Math.PI / 2.0 * wheelNonLinearity);
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / Math.sin(Math.PI / 2.0 * wheelNonLinearity);
        }

        double leftPwm, rightPwm, overPower;
        double sensitivity = 1.7;
        double angularPower;
        double linearPower;
        // Negative inertia!
        double negInertiaAccumulator = 0.0;
        double negInertiaScalar;

        if (isHighGear) {
            negInertiaScalar = 5.0;
            sensitivity = C.sensitivityHigh; // sensitivity =
                                             // Constants.sensitivityHigh.getDouble();
        } else {
            if (wheel * negInertia > 0) {
                negInertiaScalar = 2.5;
            } else {
                if (Math.abs(wheel) > 0.65) {
                    negInertiaScalar = 5.0;
                } else {
                    negInertiaScalar = 3.0;
                }
            }
            sensitivity = C.sensitivityLow; // sensitivity =
                                            // Constants.sensitivityLow.getDouble();
            if (Math.abs(throttle) > 0.1) {
                // sensitivity = 1.0 - (1.0 - sensitivity) / Math.abs(throttle);
            }
        }

        double negInertiaPower = negInertia * negInertiaScalar;
        negInertiaAccumulator += negInertiaPower;
        wheel = wheel + negInertiaAccumulator;
        if (negInertiaAccumulator > 1) {
            negInertiaAccumulator -= 1;
        } else if (negInertiaAccumulator < -1) {
            negInertiaAccumulator += 1;
        } else {
            negInertiaAccumulator = 0;
        }
        linearPower = throttle;
        // Quickturn!
        if (isQuickTurn) {
            if (Math.abs(linearPower) < 0.2) {
                double alpha = 0.1;
                quickStopAccumulator = (1 - alpha) * quickStopAccumulator + alpha * limit(wheel, 1.0) * 5;
            }
            overPower = 1.0;
            if (isHighGear) {
                sensitivity = .01;
            } else {
                sensitivity = 0.01;

            }
            angularPower = wheel;
        } else {
            overPower = 0.0;
            angularPower = Math.abs(throttle) * wheel * sensitivity - quickStopAccumulator;
            if (quickStopAccumulator > 1) {
                quickStopAccumulator -= 1;
            } else if (quickStopAccumulator < -1) {
                quickStopAccumulator += 1;
            } else {
                quickStopAccumulator = 0.0;
            }
        }
        rightPwm = leftPwm = linearPower;
        leftPwm += angularPower;
        rightPwm -= angularPower;
        if (leftPwm > 1.0) {
            rightPwm -= overPower * (leftPwm - 1.0);
            leftPwm = 1.0;
        } else if (rightPwm > 1.0) {
            leftPwm -= overPower * (rightPwm - 1.0);
            rightPwm = 1.0;
        } else if (leftPwm < -1.0) {
            rightPwm += overPower * (-1.0 - leftPwm);
            leftPwm = -1.0;
        } else if (rightPwm < -1.0) {
            leftPwm += overPower * (-1.0 - rightPwm);
            rightPwm = -1.0;
        }
        SetLeftRight(leftPwm, -rightPwm);// removed 75% modifier from SFR

    }

    /**
     * This method takes two params (left and right speed) and must be opposite to
     * go straight.
     * 
     * @param stick
     * @return yAxis
     */
    public void SetLeftRight(double LPower, double RPower) {

        right1.set(RPower * 1);
        //right2.set(RPower);
        //right3.set(RPower);
        left1.set(LPower * 1);
        //left2.set(LPower);
        //left3.set(LPower);

    }

    /**
     * This method takes in the object joystick and returns the y axis value to the
     * left most side of the gamepad.
     * 
     * @param stick
     * @return yAxis
     */
    public double getYAxisLeftSide(Joystick stick) {
        return stick.getY();
    }

    /**
     * This method takes inthe ojbect joystick and returns the y axis value to the
     * right most siide of the gamepad.
     * 
     * @param stick
     * @return
     */
    public double getYAxisRightSide(Joystick stick) {
        return stick.getThrottle();
    }

    /**
     * If the value is too small make it zero
     * 
     * @param val
     * @param deadband
     * @return value with deadband
     */
    public double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

    public void Drive(Joystick stick) {
        cheesyDrive(stick);
    }

    /**
     * If the value is too large limit it.
     * 
     * @param v
     * @param limit
     * @return value with a max limit
     */
    public static double limit(double v, double limit) {
        return (Math.abs(v) < limit) ? v : limit * (v < 0 ? -1 : 1);
    }

    /**
     * This method takes in object joystick and returns the yaxis value of the left
     * most side of the gamepad.
     * 
     * @param stick
     * @return yAxis
     */
    public double getThrottle(Joystick stick) {
        return stick.getY();
    }

	/**
	 * Initialize the compressor for self regulation of the pneumatics.
	 */
	public void InitCompressor() {
		this.compressor1.setClosedLoopControl(true);
	}

    /**
     * This method takes in the object joystick and returns the x axis value to the
     * right most side of the gamepad.
     * 
     * @param stick
     * @return xAxis
     */
    public double getWheel(Joystick stick) {
        return stick.getZ();
    }

    /**
     * This method takes in the camera value and directs the drive to turn in the direction to center it.
     * It uses the same method as the signal lights
     * TODO: Research whether we need 2nd NetworkTableInstance here??
     */
    public void TurnToTarget() {

        SmartDashboard.putNumber("LimelightXPTT", this.x);
        SmartDashboard.putNumber("LimelightVPTT", this.v);

        if (this.v >= 1){
            if (this.x > 3){
                SetLeftRight(.3,.3); //TODO: is this the correct direction?
            }
            else if (this.x <= -3){
                SetLeftRight(-.3,-.3); //TODO: is this the correct direction?
            }
            else {
                //if pointed at target, do nothing
            }
        }

    }
}
