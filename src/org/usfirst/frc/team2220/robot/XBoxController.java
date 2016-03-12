package org.usfirst.frc.team2220.robot;

import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.Joystick;

public class XBoxController extends Joystick{
	
	private Button[] buttonArray = new Button[13];
	private int joystickNumber;
	
	/*
	 * Initalize button array
	 */
	public XBoxController(int in)
	{
		super(in);
		joystickNumber = in;
		buttonArray[1] = Button.aButton;
		buttonArray[2] = Button.bButton;
		buttonArray[3] = Button.xButton;
		buttonArray[4] = Button.yButton;
		buttonArray[5] = Button.lBumper;
		buttonArray[6] = Button.rBumper;
		buttonArray[7] = Button.back;
		buttonArray[8] = Button.start;
		buttonArray[9] = Button.lStickPress;
		buttonArray[10] = Button.rStickPress; //triggers are axes
		
	}
	
	/*
	 * enum for all Button (not axes) types on xbox controller
	 */
	public enum Button {
	    aButton(1), bButton(2), xButton(3), yButton(4), lBumper(5), 
	    rBumper(6), back(7), start(8), lStickPress(9), rStickPress(10);

	    public int value;
	    public boolean[] pressed  = {false, false};
	    public boolean[] oldValue = {false, false};

	    private Button(int value) {
	      this.value = value;
	    }
	}
		
	/*
	 * true only once when button is pressed
	 */
	public boolean onPress(Button x)
	{
		return (x.pressed[joystickNumber] && !x.oldValue[joystickNumber]);
	}
	
	/*
	 * true constantly while button is held
	 */
	public boolean whileHeld(Button x) 
	{
		return x.pressed[joystickNumber];
	}
	
	/*
	 * true only once when button is released
	 */
	public boolean onRelease(Button x)
	{
		return (!x.pressed[joystickNumber] && x.oldValue[joystickNumber]);
	}
	
	public enum TriggerButton
	{
		rTrigger(0), lTrigger(1);
		
		public int value;
		public double[] currVal = {0, 0};
		public double[] oldVal  = {0, 0};
		
		private TriggerButton(int value)
		{
			this.value = value;
		}
	}
	/*
	 * true only once when TriggerButton is pressed
	 */
	public boolean onPress(TriggerButton x)
	{
		return (x.currVal[joystickNumber] > 0.85 && x.oldVal[joystickNumber] < 0.85);
	}
	
	/*
	 * true constantly while TriggerButton is held
	 */
	public boolean whileHeld(TriggerButton x) 
	{
		return x.currVal[joystickNumber] > 0.85;
	}
	
	/*
	 * true only once when TriggerButton is released
	 */
	public boolean onRelease(TriggerButton x)
	{
		return (x.currVal[joystickNumber] < 0.85 && x.oldVal[joystickNumber] > 0.85);
	}
	
	public enum POV
	{
		POV(0);
		
		private int value;
		private double oldVal = -1, currVal = -1;
		private POV(int val)
		{
			this.value = val;
		}
	}
	
	public boolean onPressPOVVal(POV x, double val)
	{
		return (x.oldVal != val) && (x.currVal == val);
	}
	
	public boolean whileHeldPOVVal(POV x, double val)
	{
		return (x.oldVal == x.currVal);
	}
	
	public boolean onReleasePOVVal(POV x, double val)
	{
		return (x.oldVal == val) && (x.currVal != val);
	}
	
	/*
	 * Call this once per robot loop
	 * This will be able to differentiate pressed, held and released
	 */
	public void update()
	{
		for(int i = 1;i <= 10;i++)
		{
			buttonArray[i].oldValue[joystickNumber] = buttonArray[i].pressed[joystickNumber];
			buttonArray[i].pressed[joystickNumber] = this.getRawButton(buttonArray[i].value);
		}
		
		TriggerButton.rTrigger.oldVal[joystickNumber] = TriggerButton.rTrigger.currVal[joystickNumber];
		TriggerButton.rTrigger.currVal[joystickNumber] = this.getRawAxis(3);
		
		TriggerButton.lTrigger.oldVal[joystickNumber] = TriggerButton.lTrigger.currVal[joystickNumber];
		TriggerButton.lTrigger.currVal[joystickNumber] = this.getRawAxis(2);
		
		POV.POV.oldVal = POV.POV.currVal;
		POV.POV.currVal = this.getPOV();
		
	}
}
