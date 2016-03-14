package org.usfirst.frc.team2220.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;


/**
 * Plays back stuff from MacroRecorder
 * @author Josh
 *
 */
public class MacroPlayer {
	Scanner scanner;
	long startTime;

	boolean onTime = true;
	double nextDouble;
	
	/**
	 * Reads a file
	 * @throws FileNotFoundException if the file doesnt exist
	 */
	public MacroPlayer() throws FileNotFoundException
	{
		//Scanner reads from the file recorded in MacroRecorder
		scanner = new Scanner(new File(MacroRecorder.autoFile));
		
		//Delimiters are commas and newlines, I think this is standard for CSV files
		scanner.useDelimiter(",|\\n");
		
		//Start Time is Current time, incase you delay or somethin
		startTime = System.currentTimeMillis();	
	}
	
	/**
	 * Plays values previously stored in the file
	 * @param drivetrain drivetrain to use
	 */
	public void play(Drivetrain drivetrain)
	{
		//Reads stored double values in order
		if ((scanner != null) && (scanner.hasNextDouble()))
		{
			double t_delta;
			
			//We never let this read faster than the values were originally stored
			if(onTime)
			{
				//Next time value
				nextDouble = scanner.nextDouble();
			}
			
			//Checks to see if we are on time
			t_delta = nextDouble - (System.currentTimeMillis()-startTime);
			
			//If we are on time, then set motor values
			if (t_delta <= 0)
			{
				//If you don't have these motors in the same order as in MacroRecorder
				//You gonna break somethin
				
				//Drive wheels
				drivetrain.setWheel(1, scanner.nextDouble());
				drivetrain.setWheel(2, scanner.nextDouble());
				drivetrain.setWheel(3, scanner.nextDouble());
				drivetrain.setWheel(4, scanner.nextDouble());
				
				//Drive modules
				drivetrain.setModuleDistanceFromStart(1, scanner.nextDouble());
				drivetrain.setModuleDistanceFromStart(2, scanner.nextDouble());
				drivetrain.setModuleDistanceFromStart(3, scanner.nextDouble());
				drivetrain.setModuleDistanceFromStart(4, scanner.nextDouble());
				//go to next double
				onTime = true;
			}
			//else don't change the values of the motors until we are "onTime"
			else
			{
				onTime = false;
			}
		}
		//end play, there are no more values to find
		else
		{
			this.end(drivetrain);
			if (scanner != null) 
			{
				scanner.close();
				scanner = null;
			}
		}
		
	}
	
	/**
	 * Stops motors and stops playing stuff
	 * @param drivetrain to stop moving
	 */
	public void end(Drivetrain drivetrain)
	{
		//Sets drivewheels to 0
		drivetrain.setWheel(1, 0);
		drivetrain.setWheel(2, 0);
		drivetrain.setWheel(3, 0);
		drivetrain.setWheel(4, 0);
		
		//TODO figure out what to do about modules when autonomous is done
		
		if (scanner != null)
		{
			scanner.close();
		}
		
	}
}
