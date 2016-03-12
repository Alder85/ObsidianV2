package org.usfirst.frc.team2220.robot;


import edu.wpi.first.wpilibj.*;

public class Drivetrain {
	private ModuleRotation flM, frM, brM, blM;
	private TwilightTalon  flW, frW, brW, blW;
	public final int frontLeft = 1, frontRight = 2, backRight = 3, backLeft = 4;
	//switch this to rotational control soon
	///////////////////
	//		         //
    // 1(FL)   2(FR) //
    //               //
    // 4(BL)   3(BR) //
    //               //
	///////////////////
	
	public Drivetrain()
	{
		
	}
	
	/**
	 * @deprecated
	 */
	public void goToDefault()
	{
		flM.goToDefault();
		frM.goToDefault();
		brM.goToDefault();
		blM.goToDefault();
	}
	/**
	 * sets the right wheels
	 */
	public void setRightWheels(double pow)
	{
		frW.set(-pow);
		brW.set(-pow);
	}
	
	/**
	 * sets the left wheels
	 */
	public void setLeftWheels(double pow)
	{
		flW.set(pow);
		blW.set(pow);
	}
	
	/**
	 * Turns modules inwards for short turning radius, or reverses outwards
	 */
	public void turnInwards()
	{
		flM.incrementQuarters(-1);
		frM.incrementQuarters(-1);
		blM.incrementQuarters(1);
		brM.incrementQuarters(1);
	}
	
	/**
	 * Turns modules outwards for wide turning radius, or 50% frame height, or transition to high frame
	 */
	public void turnOutwards()
	{
		flM.incrementQuarters(1);
		frM.incrementQuarters(1);
		blM.incrementQuarters(-1);
		brM.incrementQuarters(-1);
	}
	
	/**
	 * switch for figuring out which module
	 * maybe I want a module enum?
	 */
	public void incrementModule(int module, int turns)
	{
		switch(module)
		{
			case frontLeft:
				flM.incrementQuarters(turns);
				break;
			case frontRight:
				frM.incrementQuarters(turns);
				break;
			case backRight:
				brM.incrementQuarters(turns);
				break;
			case backLeft:
				blM.incrementQuarters(turns);
				break;
		}
	}
	
	/**
	 * Gets wheel power, for macro
	 */
	public double getWheel(int wheel)
	{
		switch(wheel)
		{
			case frontLeft:
				return flW.get();
			case frontRight:
				return frW.get();
			case backRight:
				return brW.get();
			case backLeft:
				return blW.get();
		}
		return 0;
	}
	
	/**
	 * Sets individual wheel, for macro
	 */
	public void setWheel(int wheel, double pow)
	{
		switch(wheel)
		{
			case frontLeft:
				flW.set(pow);
				break;
			case frontRight:
				frW.set(pow);
				break;
			case backRight:
				brW.set(pow);
				break;
			case backLeft:
				blW.set(pow);
				break;
		}
	}
	
	/**
	 * Gets module offset from start for macro
	 * This allows modules to start in drastically different positions from when the macro is recorded,
	 * but still work by only using the distance from start rather than the actual position
	 */
	public double getModuleDistanceFromStart(int module)
	{
		switch(module)
		{
			case frontLeft:
				return flM.distanceFromStart();
			case frontRight:
				return frM.distanceFromStart();
			case backRight:
				return brM.distanceFromStart();
			case backLeft:
				return blM.distanceFromStart();
		}
		return 0;
	}
	
	/**
	 * Sets the module to a distance relative to what was recorded
	 * for the macro
	 */
	public void setModuleDistanceFromStart(int module, double distance)
	{
		switch(module)
		{
			case frontLeft:
				flM.setDistanceFromStart(distance);
				break;
			case frontRight:
				frM.setDistanceFromStart(distance);
				break;
			case backRight:
				brM.setDistanceFromStart(distance);
				break;
			case backLeft:
				blM.setDistanceFromStart(distance);
				break;
		}
	}
	


	
	/**
	 * negative numbers for backwards
	 */
	public void incrementAllModules(int turns)
	{
		flM.incrementQuarters(turns);
		frM.incrementQuarters(turns);
		brM.incrementQuarters(turns);
		blM.incrementQuarters(turns);
	}
	
	
	/**
	 * sets up all the modules
	 * works
	 */
	public void setModules(ModuleRotation fl, ModuleRotation fr, ModuleRotation br, ModuleRotation bl)
	{
		flM = fl;
		frM = fr;
		brM = br;
		blM = bl;
	}
	
	/**
	 * sets up all the wheels
	 * 
	 * CHANGE this to wheelrotation once that class works correctly
	 */
	public void setWheels(TwilightTalon fl, TwilightTalon fr,TwilightTalon br, TwilightTalon bl)
	{
		flW = fl;
		frW = fr;
		brW = br;
		blW = bl;
	}
	
	
}
