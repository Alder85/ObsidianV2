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
	Timer timer;
	
	public Autonomous(Drivetrain inDrivetrain, Gyro inGyro)
	{
		drivetrain = inDrivetrain;
		gyro = inGyro;
		timer = new Timer();
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




























