/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;


  //TODO Clean Up Code

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


public class Robot extends TimedRobot {

  //robot map

    //NavX Specific Constants
    static final double kToleranceDegrees = 2.0f;    
    
    static final double kTargetAngleDegrees = 90.0f;

    double rotateToAngleRate;
    private double yaw;

    private static final int frontLeftMotorPort = 0;
    private static final int rearLeftMotorPort = 2;
    private static final int rearRightMotorPort = 3;
    private static final int frontRightMotorPort = 1;


    private static final int JoystickPort = 0;

    AHRS ahrs;
    //PID Constants
    //Here is the process I used for guessing the PID Gains...
    /**
     * P- Begin by starting at zero and Multiply X10 until it begins to move toward or oscilate.
     * E.X. 0 -> .0000001
     * Get as close to setpoint then one more until it oscilates
     * fine-tune
     * D- Add damping until it stops the oscillations. Begin with 1/10 of P.
     * If it Oscilates the entire time, it is too high.
     * If it does not get to the setpoint, it is too low* 
     * *Caveat... This is what worked today, but the opposite of what they say, so when all else fails, try something else.
     * I Add in (starting at 1/10 P) to smooth out any other errors.
     * 
     * This is similar (but not the same) as
     * what is described here https://trickingrockstothink.com/blog_posts/2019/10/19/tuning_pid.html
     */

    double kP = 0.009;
    double kI = 0.0005;
    double kD = 0.00;
    double desiredDistance= 100;
    double pidOutput;
    double minSpeed = -.5;
    double maxSpeed = .5;
    double encoderDistance;
    double leftSpeed;
    double currentAngle;

    private TitanQuad frontLeft = new TitanQuad(42,15600, frontLeftMotorPort);
    private TitanQuad rearLeft = new TitanQuad(42,15600, rearLeftMotorPort);
    private TitanQuad rearRight = new TitanQuad(42,15600, rearRightMotorPort);
    private TitanQuad frontRight = new TitanQuad(42,15600, frontRightMotorPort);

    private Encoder leftEncoder = new Encoder(0, 1);
    
  
 // private final TitanQuadEncoder m_encoder = new TitanQuadEncoder(rearRight, rearRightMotorPort, (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1));
  
  private Joystick stick = new Joystick(JoystickPort);

  PIDController my_pid = new PIDController(kP, kI, kD);

  private MecanumDrive m_robotDrive = new MecanumDrive( rearRight, frontRight, frontLeft,  rearLeft);

  //for turning
  PIDController turnController = new PIDController(kP, kI, kD);
  

  private boolean turnControllerEnabled = false; //since this was depreciated, create a variable to accomplish the task. 
  

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer.getInstance().startAutomaticCapture();
    frontLeft.setInverted(false);
    rearLeft.setInverted(true);
    rearRight.setInverted(true);
    //SmartDashboard.putNumber("Setpoint", desiredDistance);
    leftEncoder.setReverseDirection(true);

    ///Initialize the NavX
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
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
  }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //desiredDistance = SmartDashboard.getNumber("Setpoint", desiredDistance);
    encoderDistance=  leftEncoder.getDistance();
    //leftSpeed = leftEncoder.getRate();
    SmartDashboard.putNumber("Encoder Distance", encoderDistance);
    SmartDashboard.putNumber("left encoder", leftSpeed);

    yaw = ahrs.getYaw();
    SmartDashboard.putNumber(   "IMU_Yaw",yaw);
    
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_robotDrive.setSafetyEnabled(false);
    leftEncoder.reset();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {  
    

    

    if (stick.getRawButton(1)){
      pidOutput = my_pid.calculate(encoderDistance, desiredDistance);
      pidOutput = MathUtil.clamp(-pidOutput, -.5, .5);
      m_robotDrive.driveCartesian(0, 0, pidOutput);
       } else{
        frontLeft.set(stick.getY());
        rearLeft.set(stick.getY());
        rearRight.set(stick.getY());
        frontRight.set(stick.getY());   
       }
    //SmartDashboard.putNumber("frontLeft Encoder Distance", frontLeftEnc.getEncoderDistance());
    //SmartDashboard.putNumber("frontLeft Encoder Rate",frontLeftEnc.getRPM());
    //SmartDashboard.putNumber("Left Encoder Distance", rearLeftEnc.getEncoderDistance());
    //SmartDashboard.putNumber("Left Encoder Rate",rearLeftEnc.getRPM());
    //SmartDashboard.putNumber("Encoder Distance", m_encoder.getEncoderDistance());
    
    //SmartDashboard.putNumber("Encoder Rate", m_encoder.getSpeed());
    //SmartDashboard.putNumber("frontRight Encoder Distance", frontRightEnc.getEncoderDistance());
    //SmartDashboard.putNumber("frontRight Encoder Rate",frontRightEnc.getRPM());    
  }

  /**
   * This function is called once when teleop is enabled.
   */
  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.driveCartesian(-(stick.getRawAxis(0))/2, (stick.getRawAxis(4))/2, (stick.getRawAxis(1))/2);
    
  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledInit() {
    m_robotDrive.setSafetyEnabled(true);
  }

  /**
   * This function is called periodically when disabled.
   */
  @Override
  public void disabledPeriodic() {
  }

  /**
   * This function is called once when test mode is enabled.
   */
  @Override
  public void testInit() {
    turnController.enableContinuousInput(-180.00, 180.00);
    turnController.setTolerance(0, kToleranceDegrees);
   

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    if (stick.getRawButton(1)) {
      /*
       * While this button is held down, rotate to target angle. Since a Tank drive
       * system cannot move forward simultaneously while rotating, all joystick input
       * is ignored until this button is released.
       */
      
        //turnController.setSetpoint();
        //rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
        turnControllerEnabled = true;
      
      rotateToAngleRate = MathUtil.clamp(turnController.calculate(yaw, kTargetAngleDegrees), minSpeed, maxSpeed);
      SmartDashboard.putNumber("Target Angle",rotateToAngleRate);
      //double leftStickValue = rotateToAngleRate;
      double rightStickValue = rotateToAngleRate;
      m_robotDrive.driveCartesian(0, rightStickValue, 0);
    } else if (stick.getRawButton(2)) {
      /*
       * "Zero" the yaw (whatever direction the sensor is pointing now will become the
       * new "Zero" degrees.
       */
      ahrs.zeroYaw();
    } else if (stick.getRawButton(3)) {
      /*
       * While this button is held down, the robot is in "drive straight" mode.
       * Whatever direction the robot was heading when "drive straight" mode was
       * entered will be maintained. The average speed of both joysticks is the
       * magnitude of motion.
       */
      

      
      if (!turnControllerEnabled) {
        // Acquire current yaw angle, using this as the target angle.
        currentAngle = yaw;
        turnControllerEnabled=true;
      }
      rotateToAngleRate = MathUtil.clamp(turnController.calculate(yaw, currentAngle), minSpeed, maxSpeed);
      double magnitude = stick.getY(Hand.kLeft) / 2;
      //double leftStickValue = magnitude + rotateToAngleRate;
      double rightStickValue =  rotateToAngleRate;
      //myRobot.tankDrive(leftStickValue, rightStickValue);
      m_robotDrive.driveCartesian(-magnitude, rightStickValue, 0);
    } else {
      /* If the turn controller had been enabled, disable it now. */
      if (turnControllerEnabled) {
        turnControllerEnabled = false;
      }
      /* Standard tank drive, no driver assistance. */
      m_robotDrive.driveCartesian(-(stick.getRawAxis(0))/2, (stick.getRawAxis(4))/2, (stick.getRawAxis(1))/2);
    }
    
  }
}
