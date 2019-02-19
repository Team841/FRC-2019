package org.usfirst.frc841.lib.PathPlanner;

import java.awt.Color;

public class PathFollower {

	//Variables for acceleration, and Distance
    public double[][] accleft;
    public double[][] accright;
    public double[][] distleft;
    public double[][] distright;
    public double[][] jerkleft;
    public double[][] jerkright;
    public double[][] snapleft;
    public double[][] snapright;
    
    //Other useful variables for implementing the Control Loop
    public double period; //Seconds
    public boolean isPathFollowEnable = false;
    public boolean isPathFinished = false;

	public FalconPathPlanner path;
    
    public PathFollower(double[][] waypoints,double totalTime, double timeStep, double robotTrackWidth){
    	path = new FalconPathPlanner(waypoints);
    	//path.calculate(totalTime, timeStep, robotTrackWidth);
		 
    	calcVariables(totalTime, timeStep, robotTrackWidth);    
    }
   
   public void calcVariables(double totalTime, double timeStep, double robotTrackWidth){
	  //Calculate velocity path
	  path.calculate(totalTime, timeStep, robotTrackWidth);
	   
	  //initializes the variables
	  accleft = new double [path.smoothCenterVelocity.length][2];
	  accright = new double [path.smoothCenterVelocity.length][2];
	  distleft = new double [path.smoothCenterVelocity.length][2];
	  distright = new double [path.smoothCenterVelocity.length][2];
	  jerkleft = new double [path.smoothCenterVelocity.length][2];
	  jerkright = new double [path.smoothCenterVelocity.length][2];
	  snapleft = new double [path.smoothCenterVelocity.length][2];
	  snapright = new double [path.smoothCenterVelocity.length][2];
	  
	  
	   //This calculates all the path for acceleration and distance from the velocity curve
	   //by getting the derivative and integral of the velocity curve.
	   for (int i=0; i < (path.smoothLeftVelocity.length - 1); i++){
		   //left side acceleration vs time
		   accleft[i+1][1] = (path.smoothLeftVelocity[i+1][1] - path.smoothLeftVelocity[i][1])/timeStep;
		   accleft[i][0] = path.smoothLeftVelocity[i][0];
		   
		   //left side distance vs time
		   distleft[i+1][1] = path.smoothLeftVelocity[i][1] * timeStep + distleft[i][1];
		   distleft[i+1][0] = path.smoothLeftVelocity[i][0];
		
		   //right side acceleration vs time
		   accright[i+1][1] = (path.smoothRightVelocity[i+1][1]-path.smoothRightVelocity[i][1])/timeStep;
		   accright[i][0] = path.smoothRightVelocity[i][0];
		   
		   //right side distance vs time
		   distright[i+1][1] = path.smoothRightVelocity[i][1]* timeStep + distright[i][1];
		   distright[i+1][0] = path.smoothRightVelocity[i][0];	
	   }
	   
	   //Fill in the last bit of the array so it doesn't cause a null and exceptions. 
	   accleft[accleft.length-1][1]=0;
	   accleft[accleft.length-1][0]=accleft[accleft.length-2][0]+ timeStep;
	   distleft[distleft.length-1][1]=distleft[distleft.length-2][1];
	   distleft[accleft.length-1][0] = accleft[accleft.length-2][0]+ timeStep; 		
	
	   accright[accleft.length-1][1]=0;
	   accright[accleft.length-1][0]=accright[accright.length-2][0]+ timeStep;
	   distright[distright.length-1][1]=distright[distright.length-2][1];
	   distright[distright.length-1][0]=accright[accright.length-2][0] + timeStep;
	   distright[accright.length-1][0]=accright[accright.length-2][0]+ timeStep;

   
	   //Calculate the jerk vs time by getting the derivative of
	   //the previous calculated acceleration.
	   for (int i = 0; i < path.smoothLeftVelocity.length-1; i++) {
		   jerkleft[i+1][1] = (accleft[i+1][1] - accleft[i][1]) / timeStep;
		   jerkleft[i][0] = path.smoothLeftVelocity[i][0];
		   
		   jerkright[i+1][1] = (accright[i+1][1] - accright[i][1]) / timeStep;
		   jerkright[i][0] = path.smoothRightVelocity[i][0];		   
	   }
	   
	   jerkleft[0][1] = 0;
	   jerkright[0][1] = 0;
	   
	   //Calculate the snap vs time by getting the derivative of
	   //the previous calculated jerk.
	   for (int i = 0; i < path.smoothLeftVelocity.length-1; i++) {
		   snapleft[i+1][1] = (jerkleft[i+1][1] - jerkleft[i][1]) / timeStep;
		   snapleft[i][0] = path.smoothLeftVelocity[i][0];
		   
		   snapright[i+1][1] = (jerkright[i+1][1] - jerkright[i][1]) / timeStep;
		   snapright[i][0] = path.smoothRightVelocity[i][0];		   
	   }
	   
	   snapleft[0][1] = 0;
	   snapright[0][1] = 0;
   
   
   }   
   //To test plot
   public void plotdata(){
		FalconLinePlot fig4 = new FalconLinePlot(this.path.smoothCenterVelocity,null,Color.blue);
		fig4.yGridOn();
		fig4.xGridOn();
		fig4.setYLabel("Velocity (ft/sec)");
		fig4.setXLabel("time (seconds)");
		fig4.setTitle("Velocity Profile for Left and Right Wheels \n Left = Cyan, Right = Magenta");
		fig4.addData(this.path.smoothRightVelocity, Color.magenta);
		fig4.addData(this.path.smoothLeftVelocity, Color.cyan);
		fig4.addData(this.accleft, Color.BLUE);
		fig4.addData(this.accright, Color.BLUE);
		fig4.addData(this.distleft, Color.GREEN);
		fig4.addData(this.jerkleft, Color.WHITE);
		fig4.addData(this.jerkright, Color.WHITE);
		fig4.addData(this.snapleft, Color.YELLOW);
		fig4.addData(this.snapright, Color.YELLOW);
   }    
}
