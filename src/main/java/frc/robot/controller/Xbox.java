package frc.robot.controller;

/**
 * Contains functions for use with the XBox Controller.
 * @author Joshua Lewis joshlewi14@gmail.com
 */
public class Xbox extends ButtonMap {
//XBox Button Mapping

public Xbox() {

    this.AXIS_LEFT_X = 0;
	this.AXIS_LEFT_Y = 1;
	this.AXIS_LEFT_TRIGGER = 2;
	this.AXIS_RIGHT_TRIGGER = 3;
	this.AXIS_RIGHT_X= 4;
    this.AXIS_RIGHT_Y= 5;
    
	this.BUTTON_A = 1;
	this.BUTTON_B = 2;
	this.BUTTON_X = 3;
	this.BUTTON_Y = 4;  
	this.BUTTON_LEFT_BUMPER = 5;	
	this.BUTTON_RIGHT_BUMPER = 6;
	this.BUTTON_SHARE = 7; 			//Back Button
	this.BUTTON_OPTIONS = 8; 		//Start Button
	this.BUTTON_LEFT_JOYSTICK = 9;
	this.BUTTON_RIGHT_JOYSTICK = 10;
	   }
}