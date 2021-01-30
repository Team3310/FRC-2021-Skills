package frc.robot.controller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Contains functions for use with MOST controller (XBox, Playstation,Logitech).
 * @author Joshua Lewis joshlewi14@gmail.com
 * @author bselle
 */
public class GameController extends Joystick {

	private static final double DEADZONE = 0.1;
	private static final double TRIGGER_TOLERANCE = 0.5;

	private static final double AXIS_UP_TOLERANCE = -0.9;
	private static final double AXIS_DOWN_TOLERANCE = 0.9;
	private static final double AXIS_RIGHT_TOLERANCE = 0.9;
	private static final double AXIS_LEFT_TOLERANCE = -0.9;

	private ButtonMap map;	

	/**
	 * Constructor that creates a Joystick object.
	 */
	public GameController(int gamepadPort, ButtonMap map) {
		super(gamepadPort);
		this.map = map;
	}


	/**
	 * Returns the X position of the left stick.
	 */
	public double getLeftXAxis() {
		return getAxisWithDeadZoneCheck(getRawAxis(map.AXIS_LEFT_X));
	}

	/**
	 * Returns the X position of the right stick.
	 */
	public double getRightXAxis() {
		return getAxisWithDeadZoneCheck(getRawAxis(map.AXIS_RIGHT_X));
	}

	/**
	 * Returns the Y position of the left stick.
	 */
	public double getLeftYAxis() {
		return getAxisWithDeadZoneCheck(getRawAxis(map.AXIS_LEFT_Y));
	}

	/**
	 * Returns the Y position of the right stick.
	 */
	public double getRightYAxis() {
		return getAxisWithDeadZoneCheck(getRawAxis(map.AXIS_RIGHT_Y));
	}

	/**
	 * Checks whether Button X/A is being pressed and returns true if it is.
	 */
	public boolean getButtonStateA() {
		return getRawButton(map.BUTTON_A);
	}

	/**
	 * Checks whether Button Circle/B is being pressed and returns true if it is.
	 */
	public boolean getButtonStateB() {
		return getRawButton(map.BUTTON_B);
	}

	/**
	 * Checks whether Button Sqaure/X is being pressed and returns true if it is.
	 */
	public boolean getButtonStateX() {
		return getRawButton(map.BUTTON_X);
	}

	/**
	 * Checks whether Button Triangle/Y is being pressed and returns true if it is.
	 */
	public boolean getButtonStateY() {
		return getRawButton(map.BUTTON_Y);
	}

	
	public boolean getButtonStatePad() {
		return getRawButton(map.BUTTON_TOUCHPAD);
	}

	public int getDpadAngle() {
		return this.getPOV();
	}

	/**
	 * Returns an object of Button A.
	 */
	public Button getButtonA() {
		return new JoystickButton(this, map.BUTTON_A);
	}

	/**
	 * Returns an object of Button B.
	 */
	public Button getButtonB() {
		return new JoystickButton(this, map.BUTTON_B);
	}

	/**
	 * Returns an object of Button X.
	 */
	public Button getButtonX() {
		return new JoystickButton(this, map.BUTTON_X);
	}

	/**
	 * Returns an object of Button Y.
	 */
	public Button getButtonY() {
		return new JoystickButton(this, map.BUTTON_Y);
	}



	public JoystickButton getButtonPad() {
		return new JoystickButton(this, map.BUTTON_TOUCHPAD);
	}
	

	/**
	 * Gets the state of the Start button
	 * @return the state of the Start button
	 */
	public JoystickButton getOptionsButton(){
		return new JoystickButton(this, map.BUTTON_OPTIONS);
	}

	public JoystickButton getShareButton() {
		return new JoystickButton(this, map.BUTTON_SHARE);
	}
	
	public JoystickButton getStartButton() {
		return new JoystickButton(this, map.BUTTON_START);
	}

	public JoystickButton getLeftBumper() {
		return new JoystickButton(this, map.BUTTON_LEFT_BUMPER);
	}

	public Button getRightBumper() {
		return new JoystickButton(this, map.BUTTON_RIGHT_BUMPER);
	}

	public Button getRightTrigger() {
		return new AxisTriggerButton(this, map.AXIS_RIGHT_TRIGGER, TRIGGER_TOLERANCE);
	}
	public Button getLeftTrigger() {
		return new AxisTriggerButton(this, map.AXIS_LEFT_TRIGGER, TRIGGER_TOLERANCE);
	}

	public Button getDPadUp() {
		return new DPadTriggerButton(this, map.DPAD_UP);
	}
	public Button getDPadDown() {
		return new DPadTriggerButton(this, map.DPAD_DOWN);
	}
	public Button getDPadLeft() {
		return new DPadTriggerButton(this, map.DPAD_LEFT);
	}
	public Button getDPadRight() {
		return new DPadTriggerButton(this, map.DPAD_RIGHT);
	}
	/**
	 * Gets the state of the left stick button
	 * @return the state of the left stick button
	 */
	public JoystickButton getLeftJoystickButton() {
		return new JoystickButton(this, map.BUTTON_LEFT_JOYSTICK);
	}

	/**
	 * Gets the state of the right stick button
	 * @return the state of the right stick button
	 */
	public JoystickButton getRightJoystickButton() {
		return new JoystickButton(this, map.BUTTON_RIGHT_JOYSTICK);
	}

	/**
	 * Gets the state of the left trigger
	 * @return the state of the left trigger
	 */
	public JoystickButton getL2() {
		return new JoystickButton(this, map.BUTTON_LEFT_TRIGGER);
	}

	/**
	 * Gets the state of the right trigger
	 * @return the state of the right trigger
	 */
	public JoystickButton getR2() {
		return new JoystickButton(this, map.BUTTON_RIGHT_BUMPER);
	}
	

	private boolean inDeadZone(double input){
		boolean inDeadZone; 
		if(Math.abs(input) < DEADZONE){
			inDeadZone = true;
		}else{
			inDeadZone = false;
		}
		return inDeadZone;
	}

	private double getAxisWithDeadZoneCheck(double input){
		if(inDeadZone(input)){
			input = 0.0;       
		}
		return input; 
	}

	public double getTriggerAxis(int axis){         
		return getAxisWithDeadZoneCheck(this.getRawAxis(axis)); 
	}

	private class AxisTriggerButton extends Button {
		private GameController m_controller;
		private int m_axis;
		private double m_tolerance;

		public AxisTriggerButton(GameController controller, int axis, double tolerance) {
			m_controller = controller;
			m_axis = axis;
			m_tolerance = tolerance;
		}

		public boolean get() {
			return (m_controller.getTriggerAxis(m_axis) > m_tolerance);
		}
	}

	private class DPadTriggerButton extends Button {

		private int buttonAngle;
		private GameController  controller;

		public DPadTriggerButton(GameController controller, int dPadButtonAngle) {
			this.buttonAngle = dPadButtonAngle;
			this.controller = controller;
		}
		
		@Override
		public boolean get() {
			return controller.getDpadAngle() == buttonAngle;
		}
	}

	public boolean getLeftAxisUpTrigger(){
		return (getLeftYAxis() < AXIS_UP_TOLERANCE);
	}   

	public boolean getLeftAxisDownTrigger(){
		return (getLeftYAxis() > AXIS_DOWN_TOLERANCE);
	}   

	public boolean getLeftAxisLeftTrigger(){
		return (getLeftXAxis() > AXIS_LEFT_TOLERANCE);
	}   

	public boolean getLeftAxisRightTrigger(){
		return (getLeftXAxis() > AXIS_RIGHT_TOLERANCE);
	}

	public boolean getRightAxisUpTrigger(){
		return (getRightYAxis() < AXIS_UP_TOLERANCE);
	}   

	public boolean getRightAxisDownTrigger(){
		return (getRightYAxis() > AXIS_DOWN_TOLERANCE);
	}   

	public boolean getRightAxisLeftTrigger(){
		return (getRightXAxis() > AXIS_LEFT_TOLERANCE);
	}   

	public boolean getRightAxisRightTrigger(){
		return (getRightXAxis() > AXIS_RIGHT_TOLERANCE);
	}
	


/* 	public boolean get() {

		if (m_trigger == map.RIGHT_AXIS_UP_TRIGGER) {
			return getRightAxisUpTrigger();
		}
		else if (m_trigger == map.RIGHT_AXIS_DOWN_TRIGGER) {
			return getRightAxisDownTrigger();
		}
		else if (m_trigger == map.RIGHT_AXIS_RIGHT_TRIGGER) {
			return getRightAxisRightTrigger();
		}
		else if (m_trigger == map.RIGHT_AXIS_LEFT_TRIGGER) {
			return getRightAxisLeftTrigger();
		}
		else if (m_trigger == map.LEFT_AXIS_UP_TRIGGER) {
			return getLeftAxisUpTrigger();
		}
		else if (m_trigger == map.LEFT_AXIS_DOWN_TRIGGER) {
			return getLeftAxisDownTrigger();
		}
		else if (m_trigger == map.LEFT_AXIS_RIGHT_TRIGGER) {
			return getLeftAxisRightTrigger();
		}
		else if (m_trigger == map.LEFT_AXIS_LEFT_TRIGGER) {
			return getLeftAxisLeftTrigger();
		}
		return false;
	} */
}