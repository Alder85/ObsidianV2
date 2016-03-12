package org.usfirst.frc.team2220.robot;

import org.usfirst.frc.team2220.robot.XBoxController.*;
import com.ni.vision.NIVision;
import com.ni.vision.NIVision.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CANTalon.*;
import edu.wpi.first.wpilibj.interfaces.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import java.io.*;

public class Autonomous {
	Drivetrain drivetrain;
	Gyro gyro;
	TwilightTalon collector, rightShooter, leftShooter;
	Timer timer;
	ImageProcessor processor;
	
	
	public Autonomous(Drivetrain inDrivetrain, Gyro inGyro, TwilightTalon inCollector, TwilightTalon inRShooter, TwilightTalon inLShooter, ImageProcessor inProcessor)
	{
		drivetrain = inDrivetrain;
		gyro = inGyro;
		timer = new Timer();
		collector = inCollector;
		rightShooter = inRShooter;
		leftShooter = inLShooter;
		processor = inProcessor;
	}
	public void shoot()
	{
		double voltVal = 9;
		/*
		if(processor.getHeightDistance() > 100)
		{
			voltVal = 9;
		}
		*/
			collector.set(-1.0);
			Timer.delay(0.05);
			collector.set(0);
			rightShooter.set(-voltVal);
			leftShooter.set(voltVal);
			Timer.delay(2.25);
			collector.set(1.0);
			Timer.delay(2.0);
			rightShooter.set(0);
			leftShooter.set(0);
			collector.set(0);
	}
	
	public void lineUpToShoot()
	{
		while(true)
		{
			double temp = processor.getLeftRightDistance();
			SmartDashboard.putNumber("leftRight", temp);
			//right - left
			//if right bound positive
			//if left bound negative
			if(temp > 20 || temp < -15)
			{
				double temp2 = temp / 90;
				if(temp2 > 0.5)
					temp2 = 0.5;
				else if(temp2 < -0.5)
					temp2 = -0.5;
				drivetrain.setLeftWheels(temp2);
				drivetrain.setRightWheels(-temp2); //right reversed
				Timer.delay(0.25);
				drivetrain.setLeftWheels(0);
				drivetrain.setRightWheels(0);
			}
			else
				break;
		}
	}
	public void turnGyro(double degrees, double motorPower)
	{
		double desiredVal = gyro.getAngle() + degrees;
		double tolerance = 3;
		double correctVal = 0.1;
		if(degrees > 0)
		{
			while(gyro.getAngle() < desiredVal)
			{
					drivetrain.setRightWheels(motorPower);
					drivetrain.setLeftWheels(-motorPower);
			}		
			while(gyro.getAngle() > desiredVal)
			{
				drivetrain.setRightWheels(-motorPower + correctVal);
				drivetrain.setLeftWheels(motorPower - correctVal);
			}
		}
		else if(degrees < 0)
		{
			while(gyro.getAngle() > desiredVal)
			{
				drivetrain.setRightWheels(-motorPower);
				drivetrain.setLeftWheels(motorPower);
			}
			while(gyro.getAngle() < desiredVal)
			{
				drivetrain.setRightWheels(motorPower - correctVal);
				drivetrain.setLeftWheels(-motorPower + correctVal);
			}
		}
		drivetrain.setLeftWheels(0);
		drivetrain.setRightWheels(0);
	}
	
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
				drivetrain.setRightWheels(0);//motorPower - changeVal);
			}
			else if(tempGyro < startGyroVal - leeway)
			{
				drivetrain.setLeftWheels(0);//motorPower - changeVal);
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
}




























