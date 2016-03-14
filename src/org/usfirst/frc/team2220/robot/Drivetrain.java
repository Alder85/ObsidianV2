package org.usfirst.frc.team2220.robot;


import edu.wpi.first.wpilibj.*;

/**
 * Runs the drivetrain, simplifies commands so one can control all four wheels and all four modules very simply
 * @author Josh
 *
 */
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
	
	/**
	 * A constructor with 8 parameters would be really fun but I decided against it for now
	 */
	public Drivetrain()
	{
		
	}
	
	/**
	 * @deprecated We now assume wheels start on the ground and use that
	 */
	public void goToDefault()
	{
		flM.goToDefault();
		frM.goToDefault();
		brM.goToDefault();
		blM.goToDefault();
	}
	/**
	 * Sets the right wheels
	 * @param pow Power to set the right wheels to, reversed because they are mounted mirror to the left wheels
	 */
	public void setRightWheels(double pow)
	{
		frW.set(-pow);
		brW.set(-pow);
	}
	
	/**
	 * Sets the left wheels
	 * @param pow power to set the left wheels to
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
	 * Gets wheel power, for macro
	 * @param wheel number of the wheel to get the power from
	 * @return the power of the wheel
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
	 * @param wheel the wheel to set a power to
	 * @param pow the power to set a wheel to
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
	 * Gets module offset from start for macro.<br>
	 * This allows modules to start in drastically different positions from when the macro is recorded,
	 * but still work by only using the distance from start rather than the actual position
	 * @param module module to get the distance from
	 * @return the distance the module has moved from start
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
	 * Sets the module to a distance relative to what was recorded for the macro
	 * @param module module to set the distance on
	 * @param distance distance to go
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
	 * Increments all modules, negative numbers for backwards
	 * @param turns turns to go
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
	 * @param fl front left module
	 * @param fr front right module
	 * @param br back right module
	 * @param bl back left module
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
	 * @param fl front left wheel
	 * @param fr front right wheel
	 * @param br back right wheel
	 * @param bl back left wheel
	 */
	public void setWheels(TwilightTalon fl, TwilightTalon fr,TwilightTalon br, TwilightTalon bl)
	{
		flW = fl;
		frW = fr;
		brW = br;
		blW = bl;
	}
	
	
}
