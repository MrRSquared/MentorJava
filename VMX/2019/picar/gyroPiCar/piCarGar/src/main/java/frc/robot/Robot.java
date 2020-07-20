/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.AngleConversion;
/**
 * This is a demo program showing the use of the navX MXP to implement
 * the "rotate to angle", "zero yaw" and "drive straight" on a Tank
 * drive system.
 *
 * If Left Joystick Button 0 is pressed, a "turn" PID controller will 
 * set to point to a target angle, and while the button is held the drive
 * system will rotate to that angle (NOTE:  tank drive systems cannot simultaneously
 * move forward/reverse while rotating).
 *
 * This example also includes a feature allowing the driver to "reset"
 * the "yaw" angle.  When the reset occurs, the new gyro angle will be
 * 0 degrees.  This can be useful in cases when the gyro drifts, which
 * doesn't typically happen during a FRC match, but can occur during
 * long practice sessions.
 *
 * Finally, if Left Joystick button 2 is held, the "turn" PID controller will
 * be set to point to the current heading, and while the button is held,
 * the driver system will continue to point in the direction.  The robot 
 * can drive forward and backward (the magnitude of motion is the average
 * of the Y axis values on the left and right joysticks).
 *
 * Note that the PID Controller coefficients defined below will need to
 * be tuned for your drive system.
 */

public class Robot extends TimedRobot implements PIDOutput {
    DifferentialDrive myRobot;  // class that handles basic drive operations
    Joystick leftStick;  // set to ID 1 in DriverStation
    Joystick rightStick; // set to ID 2 in DriverStation
    AHRS ahrs;

    PIDController turnController;
	double m_rotateToAngleRate;
	double m_steering;
	

	
    
    /* The following PID Controller coefficients will need to be tuned */
    /* to match the dynamics of your drive system.  Note that the      */
    /* SmartDashboard in Test mode has support for helping you tune    */
    /* controllers by displaying a form where you can enter new P, I,  */
    /* and D constants and test the mechanism.                         */
    
    static final double kP = 0.03;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
    
    static final double kToleranceDegrees = 2.0f;    
    
    static final double kTargetAngleDegrees = 90.0f;
    
    // Channels for the wheels
    final static int leftChannel	= 0;
	final static int rightChannel	= 1;
	final static int leftReverseChannel = 8;
	final static int rightReverseChannel = 9;
	final static int turnServoChannel = 4;
    
    Victor leftMotor;
	Victor rightMotor;
	//Digital Outputs are required to reverse the motors with the piCar Motor Control
	DigitalOutput leftMotorReverse;
	DigitalOutput rightMotorReverse;
	Servo turnServo;
	AngleConversion turnConvertion;

    public Robot() {
    	leftMotor = new Victor(leftChannel);
		rightMotor = new Victor(rightChannel);
		//Setup the D.O. to reverse the motors.
		leftMotorReverse = new DigitalOutput(leftReverseChannel);
		rightMotorReverse = new DigitalOutput(rightReverseChannel);
        myRobot = new DifferentialDrive(leftMotor, rightMotor); 
        myRobot.setExpiration(0.1);
        leftStick = new Joystick(0);
		//rightStick = new Joystick(1);
		turnServo = new Servo(turnServoChannel);
		turnConvertion = new AngleConversion();
		turnConvertion.setOldRange(-1, 1);
		turnConvertion.setNewRange(-180, 180);
        try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
            ahrs = new AHRS(SPI.Port.kMXP); 
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
        turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
        turnController.setInputRange(-180.0f,  180.0f);
        turnController.setOutputRange(-1.0, 1.0);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
        turnController.disable();
        
        /* Add the PID Controller to the Test-mode dashboard, allowing manual  */
        /* tuning of the Turn Controller's P, I and D coefficients.            */
        /* Typically, only the P value needs to be modified.                   */
        //LiveWindow.addActuator(turnController, "RotateController");        
    }
    
    /**
     * Runs the motors with tank steering.
     */
    @Override
  	public void teleopPeriodic(){
       
        while (isOperatorControl() && isEnabled()) {
        	if ( leftStick.getRawButton(1)) {
        		/* While this button is held down, rotate to target angle.  
        		 * Since a Tank drive system cannot move forward simultaneously 
        		 * while rotating, all joystick input is ignored until this
        		 * button is released.
        		 */
        		if (!turnController.isEnabled()) {
        			turnController.setSetpoint(kTargetAngleDegrees);
        			m_rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
        			turnController.enable();
        		}
        		double leftStickValue = m_rotateToAngleRate;
				double rightStickValue = m_rotateToAngleRate;
				//reverse the motors if necessary.
				if (leftStickValue < 0){
					leftMotorReverse.set(true);
				}
				else{
					leftMotorReverse.set(false);
				}

				if (rightStickValue < 0){
					rightMotorReverse.set(true);
				}
				else{
					rightMotorReverse.set(false);
				}
        		myRobot.tankDrive(leftStickValue,  rightStickValue);
        	} else if ( leftStick.getRawButton(2)) {
        		/* "Zero" the yaw (whatever direction the sensor is 
        		 * pointing now will become the new "Zero" degrees.
        		 */
        		ahrs.zeroYaw();
        	} else if ( leftStick.getRawButton(3)) {
        		/* While this button is held down, the robot is in
        		 * "drive straight" mode.  Whatever direction the robot
        		 * was heading when "drive straight" mode was entered
        		 * will be maintained.  The average speed of both 
        		 * joysticks is the magnitude of motion.
        		 */
        		if(!turnController.isEnabled()) {
        			// Acquire current yaw angle, using this as the target angle.
        			turnController.setSetpoint(ahrs.getYaw());
        			m_rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
        			turnController.enable();
        		}
        		double magnitude = (leftStick.getY() + rightStick.getY()) / 2;
        		double leftStickValue = magnitude + m_rotateToAngleRate;
				double rightStickValue = magnitude - m_rotateToAngleRate;
				if (leftStickValue > 0) {

				}
        		myRobot.tankDrive(leftStickValue,  rightStickValue);
        	} else {
        		/* If the turn controller had been enabled, disable it now. */
        		if(turnController.isEnabled()) {
        			turnController.disable();
        		}
				/* Standard tank drive, no driver assistance. */
				//reverse the motors if necessary.
				if (leftStick.getY() < 0){
					leftMotorReverse.set(true);
				}
				else{
					leftMotorReverse.set(false);
				}

				if (leftStick.getX() < 0){
					rightMotorReverse.set(true);
				}
				else{
					rightMotorReverse.set(false);
				}

				turnConvertion.setInputNumber(leftStick.getX());
				m_steering = turnConvertion.getOutputNumber();
				turnServo.set(m_steering);
        		myRobot.arcadeDrive(leftStick.getY(), leftStick.getX());
        	}
            Timer.delay(0.005);		// wait for a motor update time
        }
    }

	@Override
    /* This function is invoked periodically by the PID Controller, */
    /* based upon navX MXP yaw angle input and PID Coefficients.    */
    public void pidWrite(double output) {
        m_rotateToAngleRate = output;
    }
}
