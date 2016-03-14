package org.usfirst.frc.team2220.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Simplifies collector to simple intake and outake methods
 * @author Josh
 * @deprecated This is an example of making an object where one really isn't needed because it is only used once
 */
public class Collector {
	private TwilightTalon collectorTalon;
	
	/**
	 * Collector, for collecting and decollecting balls
	 * @param talon the motor to use
	 */
	public Collector(TwilightTalon talon)
	{
		collectorTalon = talon;
	}
	
	/**
	 * Method for running the collector at a positive speed
	 * Giving this a negative speed will run it in reverse,
	 * but why do that when you can use the outtake method?
	 * @param speed The speed to go
	 */
	public void intake(double speed)
	{
		collectorTalon.set(speed);
	}
	
	/**
	 * Method for running the collector at a reverse speed
	 * Giving this a negative speed will have the same affect as the intake method
	 * 
	 * @param speed The speed to go
	 */
	public void outtake(double speed)
	{
		collectorTalon.set(-speed);
	}
	
	/**
	 * This stops the collectorTalon
	 * You could also feed a 0 into the intake or outtake functions
	 */
	public void stop()
	{
		collectorTalon.set(0);
	}
	
	/**
	 * @return speed of the talon
	 */
	public double getSpeed()
	{
		return collectorTalon.get();
	}
	/**
	 * Diagnostics of the Collector
	 * Outputs direction and speed
	 * @return String with info
	 */
	public String toString()
	{
		String out = "State: ";
		double temp = collectorTalon.get();
	
		if(temp == 0)
			out += "stopped\n";
		else if(temp > 0)
			out += "intaking at " + (temp * 100) + "% power\n";
		else if(temp < 0)
			out += "outtaking at " + (temp * -100) + "% power\n";
		return out;	
	}
}
