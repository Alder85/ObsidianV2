package org.usfirst.frc.team2220.robot;

import org.usfirst.frc.team2220.robot.XBoxController.*;
import com.ni.vision.NIVision;
import com.ni.vision.NIVision.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CANTalon.*;
import edu.wpi.first.wpilibj.interfaces.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import java.io.*;

/**
 * Autonomous utility functions.<br>
 * Piecing together these functions with different parameters allows for millions of different autonomous programs, if one is so inclined<br>
 * Currently utilizes the drivetrain, the collector and shooter<br>
 * Sensors used: Gyroscope, Camera
 * @author Josh
 *
 */
public class Autonomous {
	Drivetrain drivetrain;
	Gyro gyro;
	TwilightTalon collector, rightShooter, leftShooter, collectorExtender;
	Timer timer;
	ImageProcessorTemp processor;
	SerialCom serial;
	
	/**
	 * Constructs parts of the robot as well as a timer
	 * @param inDrivetrain drivetrain from main class
	 * @param inGyro gyro from main class
	 * @param inCollector collector talon
	 * @param inRShooter right shooter talon
	 * @param inLShooter left shooter talon
	 * @param inProcessor image processor
	 */
	public Autonomous(Drivetrain inDrivetrain, Gyro inGyro, TwilightTalon inCollector, TwilightTalon inRShooter, TwilightTalon inLShooter, ImageProcessorTemp inProcessor, SerialCom inSerial, TwilightTalon inExtender)
	{
		drivetrain = inDrivetrain;
		gyro = inGyro;
		timer = new Timer();
		collector = inCollector;
		rightShooter = inRShooter;
		leftShooter = inLShooter;
		processor = inProcessor;
		serial = inSerial;
		collectorExtender = inExtender;
	}
	
	public void extendCollector(double timeVal)
	{
		collectorExtender.set(1.0);
		Timer.delay(timeVal);
		collectorExtender.set(0);
	}
	
	public void readSerial()
	{
		for(int i = 0;i < 10;i++)
			serial.update();
	}
	
	/**
	 * Reverses collector to remove ball, spins up shooter wheels and fires
	 */
	public void shoot()
	{
		//double voltVal = (12.2204) * Math.pow((.99989), (serial.getAverageLidarValue()));
		double voltVal = 15;
		collector.set(-1.0);
		Timer.delay(0.175); //previously 0.05
		collector.set(0);
		Timer.delay(0.75);
		rightShooter.set(voltVal);
		leftShooter.set(-voltVal);
		Timer.delay(1.3);
		collector.set(1.0);
		Timer.delay(0.75);
		rightShooter.set(0);
		leftShooter.set(0);
		collector.set(0);
	}
	
	
	
	/**
	 * Uses RTL camera values to line up facing straight at the goal
	 */
	public boolean lineUpToShoot(boolean favorLeft)
	{
		Timer tempTime = new Timer();
		tempTime.start();
		while(true)
		{
			//calvin image processing
			//Timer.delay(0.1); //allows robot to settle
			double leftRightValue = 0;
			try
			{
				leftRightValue = processor.getLeftRightDistance(favorLeft);
			} catch(Exception e){}
			SmartDashboard.putNumber("leftRight", leftRightValue);
			int leftRightMid = -20;
			int leftRightRange = 20;
			
			//right - left
			//if right bound positive
			//if left bound negative
			//if you want to bias right, increase leftRightMid
			//if you want to bias left, decrease leftRightMid
			if(leftRightValue == 0) 
			{
				//look again
			}
			else if(leftRightValue > leftRightMid + leftRightRange || leftRightValue < leftRightMid - leftRightRange) //values in pixels
			{
				leftRightValue -= leftRightMid;
				double wheelMoveVal = leftRightValue * 0.003; //previously .0075
				
				double maxVal = 0.8;
				double minVal = 0.38;
				if(wheelMoveVal > maxVal)
					wheelMoveVal = maxVal;
				else if(wheelMoveVal < -maxVal)
					wheelMoveVal = -maxVal;
				if(wheelMoveVal < minVal && wheelMoveVal > -minVal)
				{
					if(wheelMoveVal < minVal)
						wheelMoveVal = minVal;
					else if(wheelMoveVal > -minVal)
						wheelMoveVal = -minVal;
				}
				
				SmartDashboard.putNumber("turnVal", wheelMoveVal);
					
				drivetrain.setLeftWheels(wheelMoveVal);
				drivetrain.setRightWheels(-wheelMoveVal); //right reversed
				Timer.delay(0.1);
				drivetrain.setLeftWheels(0);
				drivetrain.setRightWheels(0);
			}
			else
				return true;
			if(tempTime.get() > 0.5)
				break;
		}
		return false;
	}
	
	/**
	 * Lowers the frame to the ground
	 * If called before goUp(), it will actually raise the frame
	 */
	public void goDown()
	{
		drivetrain.turnInwards();
		Timer.delay(0.25);
		drivetrain.turnInwards();
		Timer.delay(0.1);
	}
	
	public void timePointTurn(double turnTime, double motorPower, boolean rightSide)
	{
		timer.reset();
		timer.start();
		while(timer.get() < turnTime)
		{
			if(rightSide)
			{
				drivetrain.setRightWheels(motorPower);
			}
			else
			{
				drivetrain.setLeftWheels(motorPower);
			}
		}
		drivetrain.setRightWheels(0);
		drivetrain.setLeftWheels(0);
	}
	
	/**
	 * Raises the frame from the ground
	 * If called after goDown(), it will actually lower the frame
	 */
	public void goUp()
	{
		drivetrain.turnOutwards();
		Timer.delay(0.1);
		drivetrain.turnOutwards();
		Timer.delay(0.1);
	}
	
	/**
	 * Turns using the gyro
	 * First takes a reading, turns a number of degrees to a point, then nudges the robot back if it overshoots
	 * @param degrees Degrees to turn, positive to turn right, negative for left
	 * @param motorPower Motor power for the inital turn, the second turn is motorPower - 0.1
	 */
	public void turnGyro(double degrees, double rightMotorPower, double leftMotorPower)
	{
		double desiredVal = gyro.getAngle() + degrees;
		double tolerance = 3;
		double correctVal = 0.1;
		double maxTime = 1.5;
		timer.reset();
		timer.start();
		if(degrees > 0)
		{
			while(gyro.getAngle() < desiredVal)
			{
					drivetrain.setRightWheels(rightMotorPower);
					drivetrain.setLeftWheels(-leftMotorPower);
					if(timer.get() > maxTime)
					{
						drivetrain.setRightWheels(0);
						drivetrain.setLeftWheels(0);
						return;
					}
						
			}		
			while(gyro.getAngle() > desiredVal)
			{
				drivetrain.setRightWheels(-rightMotorPower + correctVal);
				drivetrain.setLeftWheels(leftMotorPower - correctVal);
				if(timer.get() > maxTime)
				{
					drivetrain.setRightWheels(0);
					drivetrain.setLeftWheels(0);
					return;
				}
			}
		}
		else if(degrees < 0)
		{
			while(gyro.getAngle() > desiredVal)
			{
				drivetrain.setRightWheels(-rightMotorPower);
				drivetrain.setLeftWheels(leftMotorPower);
				if(timer.get() > maxTime)
				{
					drivetrain.setRightWheels(0);
					drivetrain.setLeftWheels(0);
					return;
				}
			}
			while(gyro.getAngle() < desiredVal)
			{
				drivetrain.setRightWheels(rightMotorPower - correctVal);
				drivetrain.setLeftWheels(-leftMotorPower + correctVal);
				if(timer.get() > maxTime)
				{
					drivetrain.setRightWheels(0);
					drivetrain.setLeftWheels(0);
					return;
				}
			}
		}
		drivetrain.setLeftWheels(0);
		drivetrain.setRightWheels(0);
	}
	/**
	 * Turns using the gyro
	 * First takes a reading, turns a number of degrees to a point, then nudges the robot back if it overshoots
	 * @param degrees Degrees to turn, positive to turn right, negative for left
	 * @param motorPower Motor power for the inital turn, the second turn is motorPower - 0.1
	 */
	public void turnToPointGyro(double degrees, double rightMotorPower, double leftMotorPower)
	{
		double desiredVal = degrees;
		double tolerance = 3;
		double correctVal = 0.1;
		double maxTime = 3.0;
		timer.reset();
		timer.start();
		if(degrees > 0)
		{
			while(gyro.getAngle() < desiredVal)
			{
					drivetrain.setRightWheels(rightMotorPower);
					drivetrain.setLeftWheels(-leftMotorPower);
					if(timer.get() > maxTime)
					{
						drivetrain.setRightWheels(0);
						drivetrain.setLeftWheels(0);
						return;
					}
						
			}		
			while(gyro.getAngle() > desiredVal)
			{
				drivetrain.setRightWheels(-rightMotorPower + correctVal);
				drivetrain.setLeftWheels(leftMotorPower - correctVal);
				if(timer.get() > maxTime)
				{
					drivetrain.setRightWheels(0);
					drivetrain.setLeftWheels(0);
					return;
				}
			}
		}
		else if(degrees <= 0)
		{
			while(gyro.getAngle() > desiredVal)
			{
				drivetrain.setRightWheels(-rightMotorPower);
				drivetrain.setLeftWheels(leftMotorPower);
				if(timer.get() > maxTime)
				{
					drivetrain.setRightWheels(0);
					drivetrain.setLeftWheels(0);
					return;
				}
			}
			while(gyro.getAngle() < desiredVal)
			{
				drivetrain.setRightWheels(rightMotorPower - correctVal);
				drivetrain.setLeftWheels(-leftMotorPower + correctVal);
				if(timer.get() > maxTime)
				{
					drivetrain.setRightWheels(0);
					drivetrain.setLeftWheels(0);
					return;
				}
			}
		}
		drivetrain.setLeftWheels(0);
		drivetrain.setRightWheels(0);
	}
	
	public void timeTurn(double turnTime, double motorPower)
	{
		timer.reset();
		timer.start();
		while(timer.get() < turnTime)
		{
			drivetrain.setRightWheels(motorPower);
			drivetrain.setLeftWheels(-motorPower);
		}
		drivetrain.setRightWheels(0);
		drivetrain.setLeftWheels(0);
	}
	
	public void pointTurnGyro(double degrees, double motorPower)
	{
		double desiredVal = gyro.getAngle() + degrees;
		double tolerance = 3;
		double correctVal = 0.1;
		double maxTimeVal = 1.5;
		timer.reset();
		timer.start();
		if(degrees > 0)
		{
			while(gyro.getAngle() < desiredVal)
			{
					drivetrain.setRightWheels(motorPower);
					drivetrain.setLeftWheels(0);
					if(timer.get() > maxTimeVal)
					{
						drivetrain.setLeftWheels(0);
						drivetrain.setRightWheels(0);
						return;
					}
			}		
			while(gyro.getAngle() > desiredVal)
			{
				drivetrain.setRightWheels(-motorPower + correctVal);
				drivetrain.setLeftWheels(motorPower - correctVal);
				if(timer.get() > maxTimeVal)
				{
					drivetrain.setLeftWheels(0);
					drivetrain.setRightWheels(0);
					return;
				}

			}
		}
		else if(degrees < 0)
		{
			while(gyro.getAngle() > desiredVal)
			{
				drivetrain.setRightWheels(0);
				drivetrain.setLeftWheels(motorPower);
				if(timer.get() > maxTimeVal)
				{
					drivetrain.setLeftWheels(0);
					drivetrain.setRightWheels(0);
					return;
				}

			}
			while(gyro.getAngle() < desiredVal)
			{
				drivetrain.setRightWheels(motorPower - correctVal);
				drivetrain.setLeftWheels(-motorPower + correctVal);
				if(timer.get() > maxTimeVal)
				{
					drivetrain.setLeftWheels(0);
					drivetrain.setRightWheels(0);
					return;
				}

			}
		}
		drivetrain.setLeftWheels(0);
		drivetrain.setRightWheels(0);
	}
	
	public void drive(double seconds, double motorPower)
	{
		timer.reset();
		timer.start();
		while(timer.get() < seconds)
		{
			drivetrain.setLeftWheels(motorPower);
			drivetrain.setRightWheels(motorPower);
		}
		drivetrain.setLeftWheels(0);
		drivetrain.setRightWheels(0);
	}
	/**
	 * Drives in a straight line, using the gyro to correct if it curves
	 * @param seconds Time to drive
	 * @param motorPower speed to drive at
	 */
	public void driveGyro(double seconds, double motorPower)
	{
		double startGyroVal = gyro.getAngle();
		double changeVal = 0.1;
		double leeway = 3;
		if(motorPower < 0)
		{
			changeVal *= -1;
		}
		timer.reset();
		timer.start();
		drivetrain.setLeftWheels(motorPower);
		drivetrain.setRightWheels(motorPower);
		while(timer.get() < seconds)
		{
			double tempGyro = gyro.getAngle();
			SmartDashboard.putNumber("gyro", tempGyro);
			SmartDashboard.putNumber("startVal", startGyroVal);
			SmartDashboard.putNumber("leeway", leeway);
			
			if(tempGyro > startGyroVal + leeway)	
			{
				drivetrain.setLeftWheels(motorPower);
				drivetrain.setRightWheels(-0.2);//motorPower - changeVal);
			}
			else if(tempGyro < startGyroVal - leeway)
			{
				drivetrain.setLeftWheels(-0.2);//motorPower - changeVal);
				drivetrain.setRightWheels(motorPower);
			}
			else
			{
				drivetrain.setLeftWheels(motorPower);
				drivetrain.setLeftWheels(motorPower);
			}
		}
		drivetrain.setLeftWheels(0);
		drivetrain.setRightWheels(0);
	}
	/**
	 * Moves forward and backward to line up based on camera distance, eventually will use LIDAR
	 * @deprecated use lidar instead
	 */
	public void forwardBackwardToShoot()
	{
		while(true)
		{
			Timer.delay(0.1);
			//processor.lookForTarget();
			if(processor.getHeightDistance() > 170) //higher = closer
			{
				drivetrain.setLeftWheels(-0.5);
				drivetrain.setRightWheels(-0.5);
				Timer.delay(0.1);
			}
			else if(processor.getHeightDistance() < 160) //lower = farther
			{
				drivetrain.setLeftWheels(0.5);
				drivetrain.setRightWheels(0.5);
				Timer.delay(0.1);
			}
			else
				break;
			drivetrain.setLeftWheels(0);
			drivetrain.setRightWheels(0);
		}
	}
	
}




























