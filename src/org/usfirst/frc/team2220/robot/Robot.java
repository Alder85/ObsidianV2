
package org.usfirst.frc.team2220.robot;


//import edu.wpi.first.wpilibj.*;
import org.usfirst.frc.team2220.robot.XBoxController.Button;
import org.usfirst.frc.team2220.robot.XBoxController.TriggerButton;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.FileNotFoundException;
import java.io.IOException;

/**
 * This is 2220's test code
 */
//potatoe potatoe 
public class Robot extends SampleRobot {
    SmartDashboard dashboard;

    public Robot() {
        
    }

    /**
     * Autonomous 
     * @TODO
     */
    public void autonomous() {
       
    }

    public double deadZone(double val, double zone)
    {
    	if(val < zone && val > -zone)
    		return 0;
    	return val;
    }
    
    TwilightTalon talon7 = new TwilightTalon(7);
	ModuleRotation frModule = new ModuleRotation(talon7);
	
	TwilightTalon talon2 = new TwilightTalon(2);
	ModuleRotation flModule = new ModuleRotation(talon2);
	
	TwilightTalon talon4 = new TwilightTalon(4);
	ModuleRotation blModule = new ModuleRotation(talon4);
	
	TwilightTalon talon5 = new TwilightTalon(5);
	ModuleRotation brModule = new ModuleRotation(talon5);
	
	SmartDashboard dash = new SmartDashboard();
	//TwilightTalon[] talons = new TwilightTalon[16];
	TwilightTalon frWheel = new TwilightTalon(8);
	TwilightTalon flWheel = new TwilightTalon(1);
	TwilightTalon blWheel = new TwilightTalon(3);
	TwilightTalon brWheel = new TwilightTalon(6);
	
	
	
	
	
	TwilightTalon collector = new TwilightTalon(9);
	TwilightTalon rightShooter = new TwilightTalon(10);
	TwilightTalon leftShooter  = new TwilightTalon(12);
	
	//TwilightTalon lifterRelease = new TwilightTalon(13);
	//TwilightTalon wench = new TwilightTalon(14);
	
	//POSITIVE SPINS counterclockwise 
	TwilightTalon collectorExtender = new TwilightTalon(11);
	//counterclockwise for backward
	//clockwise for forwards
	//potatoes are potatoes
	
	Drivetrain drivetrain = new Drivetrain();
	
	
	
	//LEFt need to be flipped
	//xbox start 0,0 top left so flip right
	XBoxController driverController = new XBoxController(0);
	XBoxController manipulatorController = new XBoxController(1);
	
	
	DigitalInput frontCollector = new DigitalInput(1);
	DigitalInput rearCollector  = new DigitalInput(2);
	
	
	Timer resetTimer = new Timer();
	
 	//testModule.changeControlMode(TalonControlMode.Position);
	//testModule.setFeedbackDevice(FeedbackDevice.EncFalling);
	
	
    /**
     * TeleOp
     * @TODO
     */
    public void operatorControl() {
    	
    	//don't declare stuff here
    	
    	collectorExtender.enableBrakeMode(true);
    	
    	frWheel.setMaxCurrent(100);
    	flWheel.setMaxCurrent(100);
    	brWheel.setMaxCurrent(100);
    	blWheel.setMaxCurrent(100);
    	
    	collector.setMaxCurrent(60);
    	rightShooter.setMaxCurrent(70);
    	leftShooter.setMaxCurrent(70);
    	//talon2.setMaxCurrent(5);
    	
    	collectorExtender.setMaxCurrent(120);
    	//lifterRelease.setMaxCurrent(30);
    	
    	flModule.reverseTalon(true);
    	blModule.reverseTalon(true);

    	frModule.reverseTalon(false);
    	brModule.reverseTalon(true);
    	
    	frModule.setRightWheel(true);
    	brModule.setRightWheel(true);
    	
    	frModule.reverseSensor(false);
    	brModule.reverseSensor(false);
    	
    	talon2.enableBrakeMode(false);
    	talon4.enableBrakeMode(false);
    	talon5.enableBrakeMode(false);
    	talon7.enableBrakeMode(false);
    	

    	
    	
    	double allTuning = 2.5;
    	
    	frModule.setP(allTuning);
    	brModule.setP(allTuning);
    	flModule.setP(allTuning);
    	blModule.setP(allTuning);
    	
    	drivetrain.setModules(flModule, frModule, brModule, blModule);
    	drivetrain.setWheels(flWheel, frWheel, brWheel, blWheel);
    	/*
    	lifterRelease.setFeedbackDevice(FeedbackDevice.PulseWidth);
    	lifterRelease.changeControlMode(TalonControlMode.Position);
    	lifterRelease.setPID(1.0, 0.001, 0.0);
    	lifterRelease.reverseOutput(true);
    	double lifterPosition = (lifterRelease.getPulseWidthPosition() / 4096);
    	
    	lifterRelease.setAllowableClosedLoopErr(30);
    	*/
    	double leftAxis, rightAxis;
    	double wenchAxis;
    	double[] maxVal = new double[4];
    	double wheelDZ = 0.15;
    	double[] temp = new double[4];
    	double tempTune;
    	int dashCount = 0;
    	
        while (isOperatorControl() && isEnabled()) {
			/////////////////////////
			//  Primary Controller //
			/////////////////////////
        	driverController.update();
        	
			/////////////////////////
			//       Modules       //
			/////////////////////////
        	if(driverController.onPress(TriggerButton.lTrigger))
        		drivetrain.incrementAllModules(-1);
        	
        	if(driverController.onPress(TriggerButton.rTrigger))
        		drivetrain.incrementAllModules(1);
        	
        	if(driverController.onPress(Button.lBumper))
        		drivetrain.turnOutwards();
        	
        	if(driverController.onPress(Button.rBumper))
        		drivetrain.turnInwards();
        	
        	if(driverController.whileHeld(Button.aButton))
        	{
        		tempTune = 1;
        		frModule.setP(tempTune);
        		flModule.setP(tempTune);
        		blModule.setP(tempTune);
        		brModule.setP(tempTune);
        	}
        	else
        	{
        		tempTune = allTuning;
        		frModule.setP(tempTune);
        		flModule.setP(tempTune);
        		blModule.setP(tempTune);
        		brModule.setP(tempTune);
        	}
        	dash.putNumber("tempTune", tempTune);
        	
			/////////////////////////
			//     Drive Wheels    //
			/////////////////////////
        	leftAxis = deadZone(driverController.getRawAxis(1), wheelDZ);
        	rightAxis = deadZone(driverController.getRawAxis(5) * -1, wheelDZ);
        	
        	drivetrain.setLeftWheels(leftAxis);
        	drivetrain.setRightWheels(rightAxis);
        	
			//////////////////////////
			// Secondary Controller //
			//////////////////////////
			manipulatorController.update();
			
			/////////////////////////
			//      Collector      //
			/////////////////////////
			
			if(manipulatorController.whileHeld(TriggerButton.lTrigger))
			{
				//if(rearCollector.get())
				//{
					collector.set(1.0);
				//}
				//else
					collector.set(0);
			}
			else if(manipulatorController.whileHeld(TriggerButton.rTrigger))
			{
				//if(rearCollector.get())
				//{
					collector.set(-1.0);
				//}
				//else
					collector.set(0);
			}
			else
				collector.set(0);
			
			/////////////////////////
			//    Shooter Wheels   //
			/////////////////////////
			if(manipulatorController.whileHeld(Button.aButton))
			{
        		rightShooter.set(1.0);
        		leftShooter.set(-1.0);
        	}
        	else
        	{
        		rightShooter.set(0);
        		leftShooter.set(0);
        	}

			/////////////////////////
			//  Collector Extender //
			/////////////////////////
			/*
			if(manipulatorController.getPOV() != -1)
			{
				if(manipulatorController.getPOV() == 0 || manipulatorController.getPOV() == 315 || manipulatorController.getPOV() == 45)
				{
					if(frontCollector.get())
	        			collectorExtender.set(-1.0);
	        		else
	        			collectorExtender.set(0);
				}
				else if(manipulatorController.getPOV() == 135 || manipulatorController.getPOV() == 180 || manipulatorController.getPOV() == 225)
				{
					if(rearCollector.get())
	        			collectorExtender.set(1.0);
	        		else
	        			collectorExtender.set(0);
				}
				else
	        	{
	        		collectorExtender.set(0);
	        	}
			}
			else
        	{
        		collectorExtender.set(0);
        	}
        	*/
			
			/////////////////////////
			//        Lifter       //
			///////////////////////// TODO make it so you can only press this button once
			/*
			if(manipulatorController.onPress(Button.bButton))
			{
				if(rearCollector.get())
				{
					lifterRelease.set(lifterRelease.get() - 0.1875);
				}
			}
			
			if(manipulatorController.onPress(Button.yButton))
			{
				if(rearCollector.get())
				{
					lifterRelease.set(lifterRelease.get() + 0.1875);
				}
			}
			
			wenchAxis = deadZone(manipulatorController.getRawAxis(1) * -1, wheelDZ);
        	wench.set(wenchAxis);
        	*/
			       	
        	/////////////////////////
        	//   Print Everything  //
        	/////////////////////////
        	dashCount++;
        	if(dashCount > 20000)
        		dashCount = 0;
        	if(dashCount % 10 == 0)
        	{
	        	dash.putBoolean("frontCollector", !frontCollector.get());
	        	dash.putBoolean("rearCollector", !rearCollector.get());
	        	
	        	dash.putNumber("backRightErr", talon5.getError());
	        	dash.putNumber("backLeftErr", talon4.getError());
	        	dash.putNumber("frontRightErr", talon7.getError());
	        	dash.putNumber("frontLeftErr", talon2.getError());
	        	
	        	dash.putNumber("brPos", brModule.getDoublePosition());
	        	dash.putNumber("blPos", blModule.getDoublePosition());
	        	dash.putNumber("frPos", frModule.getDoublePosition());
	        	dash.putNumber("flPos", flModule.getDoublePosition());
	        	
	        	dash.putNumber("pos2", brModule.getModuloPosition());
	        	dash.putNumber("pos3", brModule.getDesiredPosition());
	
	        	dash.putBoolean("withinStopBR", brModule.withinRange());
	        	dash.putBoolean("withinStopBL", blModule.withinRange());
	        	dash.putBoolean("withinStopFR", frModule.withinRange());
	        	dash.putBoolean("withinStopFL", flModule.withinRange());
	        	
	        	
	        	dash.putNumber("powBR", talon5.getOutputCurrent());
	        	dash.putNumber("powBL", talon4.getOutputCurrent());
	        	dash.putNumber("powFR", talon7.getOutputCurrent());
	        	dash.putNumber("powFL", talon2.getOutputCurrent());
	        	
	        	dash.putNumber("voltageBR", talon5.getOutputVoltage());
	        	dash.putNumber("voltageBL", talon4.getOutputVoltage());
	        	dash.putNumber("voltageFR", talon7.getOutputVoltage());
	        	dash.putNumber("voltageFL", talon2.getOutputVoltage());
	        	
	        	dash.putBoolean("disabledBR", talon5.isDisabled());
	        	dash.putBoolean("disabledBL", talon4.isDisabled());
	        	dash.putBoolean("disabledFR", talon7.isDisabled());
	        	dash.putBoolean("disabledFL", talon2.isDisabled());
	        	
	        	dash.putNumber("leftShooterCurrent", leftShooter.getOutputCurrent());
	        	dash.putNumber("rightShooterCurrent", rightShooter.getOutputCurrent());
        	
        	
				/////////////////////////
				//     Max Currents    //
				/////////////////////////
	        	temp[0] = talon5.getOutputCurrent();
	        	if(temp[0] > maxVal[0])
	        		maxVal[0] = temp[0];
	        	dash.putNumber("maxBR", maxVal[0]);
	        	
	        	temp[1] = talon4.getOutputCurrent();
	        	if(temp[1] > maxVal[1])
	        		maxVal[1] = temp[1];
	        	dash.putNumber("maxBL", maxVal[1]);
	        	
	        	temp[2] = talon7.getOutputCurrent();
	        	if(temp[2] > maxVal[2])
	        		maxVal[2] = temp[2];
	        	dash.putNumber("maxFR", maxVal[2]);
	        	
	        	temp[3] = talon2.getOutputCurrent();
	        	if(temp[3] > maxVal[3])
	        		maxVal[3] = temp[3];
	        	dash.putNumber("maxFL", maxVal[3]);
        	
        	}
			/////////////////////////
			//   Test All Modules  //
			/////////////////////////
        	frWheel.test();
        	flWheel.test();
        	blWheel.test();
        	brWheel.test();

        	talon7.test();
        	talon2.test();
        	talon4.test();
        	
        	talon5.test();
        	
        	
        	
        	
        	if((talon7.isDisabled() || talon2.isDisabled() || talon4.isDisabled() || talon5.isDisabled()))
        	{
        		//resetTimer.start();
        		talon7.disable();
        		talon2.disable();
        		talon4.disable();
        		talon5.disable();
        	}
        	
        	if(manipulatorController.onPress(Button.back))
        	{
        		//resetTimer.stop();
        		//resetTimer.reset();
        		talon7.enable();
        		talon2.enable();
        		talon4.enable();
        		talon5.enable();
        		frModule.resetTarget();
        		flModule.resetTarget();
        		blModule.resetTarget();
        		brModule.resetTarget();
        	}
        	if(manipulatorController.onPress(Button.start))
        	{
        		talon7.disable();
        		talon2.disable();
        		talon4.disable();
        		talon5.disable();
        	}
        	
        	
        	collector.test();
        	//rightShooter.test(); //except these ones i guess
        	//leftShooter.test();
        	
        	//lifterRelease.test();
        	collectorExtender.test();
        	
       

            Timer.delay(0.005);		// wait for a motor update time
        }
    }
    public void practice()
    {
    	while(isEnabled())
    	{
	    	dash.putNumber("backRightErr", talon5.getError());
	    	dash.putNumber("backLeftErr", talon4.getError());
	    	dash.putNumber("frontRightErr", talon7.getError());
	    	dash.putNumber("frontLeftErr", talon2.getError());
    	}
    }

    /**
     * Test mode
     * @TODO diagnostics
     */
    public void test() {
    	
    }
}
