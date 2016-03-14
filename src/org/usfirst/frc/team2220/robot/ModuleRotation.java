package org.usfirst.frc.team2220.robot;

import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;

/**
 * Rotation for modules using abosulte encoders plugged into CANTalons
 * @author Josh
 *
 */
public class ModuleRotation {
	private TwilightTalon talon;
	double offsetTarget = 0.0, target, startPos;
	int relativeAllowableError = 30, allowableError = 10;
	boolean sensorReversed = false;
	boolean isRightWheel = false;
	
	/**
	 * Sets the inital position to the current position, assumes the modules start on the ground
	 * @param Talon talon to use
	 */
	public ModuleRotation(TwilightTalon Talon) {
		talon = Talon;
		talon.setFeedbackDevice(FeedbackDevice.PulseWidth);
		talon.changeControlMode(TalonControlMode.Position);
		
		target = getDoublePosition();
		startPos = target;
		this.reverseSensor(false);
		
		talon.setProfile(0); //we aren't using multiple profiles yet
		
		this.setI(0.001);
		talon.setAllowableClosedLoopErr(allowableError);	//how much error is allowable
	}
	
	/**
	 * Resets modules to current position if they reset
	 */
	public void resetTarget()
	{
		target = getDoublePosition();
	}
	
	/**
	 * Gets relative distance from start, so modules can start differently from when macro was recorded, for the macro
	 * @return the distance from the starting position
	 */
	public double distanceFromStart()
	{
		return target - startPos;
	}
	
	/**
	 * Receives a value previously taken from distanceFromStart(), then adds in to the startPos. for the Macro
	 * @param in the distance from start to go to
	 */
	public void setDistanceFromStart(double in)
	{
		talon.set(startPos + in);
	}
	
	/**
	 * This is a bit complicated, but essentially on the drivetrain when motors are mounted right side versus left side
	 * they are mirrored, and simply reversing talon or sensor direction isn't always enough to get them to function
	 * properly, especially with the variable startup position the Mechanical team requested.
	 * This will reverse some commands that go to the rightWheel, relative to quarterTurns
	 * I may or may not have to use this for the macro as well.
	 * @param in whether of not this wheel is a right wheel
	 */
	void setRightWheel(boolean in)
	{
		isRightWheel = in;
	}
	/**
	 * increments the motor position by an eighth-turn. inputting 2 gives a quarter turn, etc.<br>
	 * Technically we refer to an 8th full turn as a quarter turn, as only 1/2 full is relevant 
	 * because the modules are symmetrical
	 * @param quarters the amount of quarter turns to go
	 */
	void incrementQuarters(int quarters) {		
		//target = talon.get() + (quarters * 0.125);
		//talon.enable();
		if(talon.isEnabled())
		{
			if(!isRightWheel)
				target += quarters * 0.125;
			else
				target -= quarters * 0.125;
			talon.set(target);
		}
	}	
	
	/**
	 * We don't need this anymore!
	 * We just assume the wheel start flat and use that as the offset.
	 * @deprecated
	 */
	void setOffset(double val)
	{
		offsetTarget = val;
	//	target = offsetTarget;
	}
	
	/**
	 * We don't really use this because of the startup assumption that the wheels are flat
	 * @deprecated
	 */
	void goToDefault()
	{
		//talon.setPulseWidthPosition((int) this.getPosition());
		//talon.setPosition(talon.getPosition() % 1);
		
		target = offsetTarget + getDesiredPosition();
	//	talon.set(target);
	}
	
	
	
	/**
	 * Returns absolute position 0 to 4095, does not loop, so technically (-infinity, infinity)
	 * uncomment constant to convert to degrees
	 * uncomment 0xFFF to loop so you only get 0 to 4095
	 * @return the current position
	 */
	double getPosition() 
	{		
		if(sensorReversed)
			return talon.getPulseWidthPosition() * -1;
		return talon.getPulseWidthPosition();// & 0xFFF)/***0.087890625*/;
	}
	
	/**
	 * Scales 0 to 4095 to 0 to 1
	 * @return the current position as a double
	 */
	double getDoublePosition() 
	{
		return this.getPosition() / 4096;
	}
	
	/**
	 * Gets distance from last full turn
	 * @return the current position relative to the last full turn
	 */
	double getModuloPosition() 
	{
		return this.getDoublePosition() % 1;
	}
	
	/**
	 * Figures out which was to turn to be flat
	 * @return how far you must turn to make the modules be flat
	 */
	double getDesiredPosition()
	{
		double out = getModuloPosition();
		if(out <= 0.25)
			out = 0;
		else if(out >= 0.75)
			out = 1;
		else
			out = 0.5;
		return out + getIntegerPosition();
	}
	
	/**
	 * Integer position, as a double
	 * @return the integer position, in full turns
	 */
	double getIntegerPosition() {
		 return this.getDoublePosition() - this.getModuloPosition();
	}
	
	/**
	 * sets the PID
	 * @param p proportional constant
	 * @param i integral constant
	 * @param d derivative constant
	 */
	void setPID(double p, double i, double d)
	{
		talon.setPID(p, i, d);
	}
	/**
	 * @param in proportional constant
	 */
	void setP(double in)
	{
		talon.setP(in);
	}
	/**
	 * @param in integral constant
	 */
	void setI(double in)
	{
		talon.setI(in);
	}
	
	/**
	 * @param in derivative constant
	 */
	void setD(double in)
	{
		talon.setD(in);
	}

	
	/**
	 * Reverses the sensor
	 * @param reversed whether or not the sensor is reversed
	 */
	void reverseSensor(boolean reversed)
	{
		talon.reverseSensor(reversed);
		sensorReversed = reversed;
	}
	
	/**
	 * reverses the talon
	 * @param reversed which way to reversed it
	 */
	void reverseTalon(boolean reversed)
	{
		talon.reverseOutput(reversed);
	}
	
	/**
	 * @return If the module is within the allowable error
	 */
	boolean withinRange()
	{
		return (this.getError() < relativeAllowableError && this.getError() > -relativeAllowableError);
	}


	/**
	 * @param point sets the talon to this point
	 */
	void set(double point)
	{
		talon.set(point);
	}
	
	/**
	 * @return the error of the PID loop
	 */
	double getError() {			
		return talon.getError();
	}
	
	void stop() {			
		talon.stop();
	}
	
	public String toString() {	
		return "";
	}
	
	
}
