package org.usfirst.frc.team2220.robot;

import java.io.FileWriter;
import java.io.IOException;

/**
 * 2016 Macro, based off of last years at https://github.com/DennisMelamed/FRC-Play-Record-Macro
 * Stores data from motors
 * TODO add more motors than just drivetrain
 * @author Josh
 *
 */
public class MacroRecorder {
	//this object writes values into the file we specify
		FileWriter writer;
		
		long startTime;
		//Number of file we're dealing with
		static final int autoNumber = 10;
		//only call this, makes sure we are reading/writing from the same file.
		static final String autoFile = new String("/home/lvuser/recordedAuto" + autoNumber + ".csv");
		
		/**
		 * Starts a timer and creates a file writer
		 * @throws IOException if you try to write files wrong
		 */
		public MacroRecorder() throws IOException
		{
				//Start Time
				startTime = System.currentTimeMillis();
				
				//put the filesystem location you are supposed to write to as a string 
				writer = new FileWriter(autoFile);
		}
		

		/**
		 * Records values and stores them to a file
		 * @param drivetrain drivetrain to store the values from
		 * @throws IOException if you try to write files wrong
		 */
		public void record(Drivetrain drivetrain) throws IOException
		{
			if(writer != null)
			{
				//Stores a time index associated with these values
				writer.append("" + (System.currentTimeMillis()-startTime));
				
				//Values on this line associated with a time go after this
				
				//drive motors, standard vBus power
				writer.append("," + drivetrain.getWheel(1));
				writer.append("," + drivetrain.getWheel(2));
				writer.append("," + drivetrain.getWheel(3));		
				writer.append("," + drivetrain.getWheel(4));
				
				//drive modules, using setpoints
				writer.append("," + drivetrain.getModuleDistanceFromStart(1));
				writer.append("," + drivetrain.getModuleDistanceFromStart(2));
				writer.append("," + drivetrain.getModuleDistanceFromStart(3));
				writer.append("," + drivetrain.getModuleDistanceFromStart(4) + "\n");
				
				/*
				 * Gotta have the \n delimiter on the end of this set of data otherwise nosuchelementexception
				 */
			}
		}
		
		
		/**
		 * Stops everything and puts it in a file
		 * @throws IOException if you try to write files wrong
		 */
		public void end() throws IOException
		{
			if(writer !=null)
			{
				writer.flush();
				writer.close();
			}
		}
}
