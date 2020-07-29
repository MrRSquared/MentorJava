/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Import Drive Code
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//import encoder
import edu.wpi.first.wpilibj.Encoder;

//import PID
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpiutil.math.MathUtil;

//import the Pixy
import io.github.pseudoresonance.pixy2api.*;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.I2CLink;
//import the Custom PixyPID
import frc.robot.PixyPID;
//import our conversion class
import frc.robot.Conversion;

//ArrayList for the pixy block arrays
import java.util.ArrayList;
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

    //NavX Specific Constants
  static final double kToleranceDegrees = 2.0f;    
    
  static final double kTargetAngleDegrees = 90.0f;

  double rotateToAngleRate;
  private double yaw;

  //Setup Constants
  private static final int leftMotorPort = 0;
  private static final int rightMotorPort = 1;
  private static final int leftEncPortA =  0;
  private static final int leftEncPortB = 1;
  private static final int rightEncPortA = 2;
  private static final int rightEncPortB = 3;
  private static final int joystickPort = 0;

  private final XboxController stick = new XboxController(joystickPort);

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

  double kP = .09;
  double kI = 0.001;
  double kD = .004;
  double desiredSpeed = 100;
  double pidOutput;
  double minSpeed = -.6;
  double maxSpeed =.6;
    

  private final PWMVictorSPX m_leftMotor = new PWMVictorSPX(leftMotorPort);
  private final PWMVictorSPX m_rightMotor = new PWMVictorSPX(rightMotorPort);
  private final DifferentialDrive myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
  //private final Joystick m_stick = new Joystick(joystickPort);
  

  private final Encoder leftEncoder = new Encoder(leftEncPortA, leftEncPortB);
  private final Encoder rightEncoder = new Encoder(rightEncPortA, rightEncPortB);
  private double leftSpeed;
  private double rightSpeed;
  private double leftDistance;
  private double rightDistance;
  PIDController my_pid = new PIDController(kP, kI, kD);

    //for turning
  PIDController turnController = new PIDController(kP, kI, kD);
  private boolean turnControllerEnabled = false; //since this was depreciated, create a variable to accomplish the task. 

  //Pixy Instantiation

  private Pixy2 pixycam;
  boolean isCamera = false;
  int state = -1;
  // Begin PID piece for Pixy
  private int panDesiredPosition = 500; // this is the default position (Pixy calls it m_command so as to use it for
                                        // velocity too)
  private int tiltDesiredPosition = 500;
  private int panOffset;
  private int tiltOffset;

  private PixyPID panPID;
  private PixyPID tiltPID;

  // Drive logic for following the target.

  // PID Constants
  private int panP = 400;
  private int panI = 0;
  private int panD = 600;

  private int tiltP = 500;
  private int tiltI = 0;
  private int tiltD = 700;


  @Override
  public void robotInit() {

    // PIDController my_pid = new PIDController(kP, kI, kD);
   SmartDashboard.putNumber("P", kP);
   SmartDashboard.putNumber("I", kI);
   SmartDashboard.putNumber("D", kD);
   SmartDashboard.putNumber("Setpoint", desiredSpeed);

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
  //Pixy
  pixycam = Pixy2.createInstance(new I2CLink()); // Creates a new Pixy2 camera using SPILink
  pixycam.init(); // Initializes the camera and prepares to send/receive data
  panPID = new PixyPID();
  panPID.setPIDs(panP, panI, panD);
  tiltPID = new PixyPID();
  tiltPID.setPIDs(tiltP, tiltI, tiltD);
  }

  @Override
  public void robotPeriodic() {
    leftSpeed = leftEncoder.getRate();
    rightSpeed = rightEncoder.getRate();
    
    leftDistance = leftEncoder.getDistance();
    rightDistance = rightEncoder.getDistance();
    SmartDashboard.putNumber("Left Speed", leftSpeed);
    SmartDashboard.putNumber("Right Speed", rightSpeed);
    SmartDashboard.putNumber("Left Distance", leftDistance);
    SmartDashboard.putNumber("right Distance", rightDistance);
    SmartDashboard.putNumber("pid Output", pidOutput);
    desiredSpeed = SmartDashboard.getNumber("Setpoint", desiredSpeed);
    kP =SmartDashboard.getNumber("P", kP);
    kI = SmartDashboard.getNumber("I", kI);
    kD = SmartDashboard.getNumber("D", kD);
    my_pid.setP(kP);
    my_pid.setI(kI);
    my_pid.setD(kD);
    SmartDashboard.putNumber("NewkP", kP);
    yaw = ahrs.getYaw();
    SmartDashboard.putNumber(   "IMU_Yaw",yaw);
  }

  @Override
  public void autonomousInit() {
    rightEncoder.reset();
    leftEncoder.reset();
   
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
        //double rightSpeed =  rightEncoder.getRate();
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    pidOutput = my_pid.calculate(leftEncoder.getDistance(), desiredSpeed);
    pidOutput = MathUtil.clamp(-pidOutput, -.6, .6);

    myRobot.arcadeDrive(pidOutput, 0);
    
  }

  @Override
  public void teleopInit() {

    //turn on the lights
    //pixycam.setLamp((byte) 1, (byte) 1);
    //Center the servos
    pixycam.setServos(500, 500);
  }

  @Override
  public void teleopPeriodic() {
    myRobot.arcadeDrive(stick.getY(), stick.getX());
  
  //SmartDashboard.putBoolean( "Camera" , isCamera); //publish if we are connected
   pixycam.getCCC().getBlocks( false , 255 , 255 ); //run getBlocks with arguments to have the camera
    //SmartDashboard.putNumber( "Stuff" , pixycam.getCCC().getBlocks() );
  //acquire target data
    ArrayList<Block> blocks = pixycam.getCCC().getBlockCache(); //assign the data to an ArrayList for convinience
    if (blocks.size() > 0 )
      {
        //Get the blocks
       // String data = blocks.get( 0 ).toString(); // string containing target info
       // double xcoord = blocks.get( 0 ).getX(); // x position of the largest target
        //double ycoord = blocks.get( 0 ).getY(); // y position of the largest target
        
        //Calculate error for pan
        panOffset = pixycam.getFrameWidth()/2 - blocks.get(0).getX();
        tiltOffset = pixycam.getFrameHeight()/2 - blocks.get(0).getY();
        //Send the error to the PID for it to work its magic
        panPID.setError(panOffset);
        tiltPID.setError(tiltOffset);
        panPID.calculateMyPID();
        tiltPID.calculateMyPID();
        //Retrive the desired position (Pan)
        panDesiredPosition = panPID.getDesiredPosition();
        tiltDesiredPosition = tiltPID.getDesiredPosition(); 

        pixycam.setServos(panDesiredPosition, tiltDesiredPosition);
      }
      SmartDashboard.putBoolean( "present" , true ); // show there is a target present
      SmartDashboard.putNumber( "pan offset" ,panOffset);
      //SmartDashboard.putNumber( "Ycoord" , ycoord);
      //SmartDashboard.putString( "Data" , data );
      SmartDashboard.putNumber( "pan" , panDesiredPosition );
      SmartDashboard.putNumber("blocks", blocks.size());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

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
      double leftStickValue = rotateToAngleRate;
      double rightStickValue = -rotateToAngleRate;
      myRobot.tankDrive(leftStickValue, rightStickValue);
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
        turnController.setSetpoint(ahrs.getYaw());
        rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
        turnControllerEnabled=true;
      }
      rotateToAngleRate = MathUtil.clamp(turnController.calculate(ahrs.getYaw()), minSpeed, maxSpeed);
      double magnitude = stick.getY(Hand.kLeft) + stick.getY(Hand.kRight) / 2;
      double leftStickValue = magnitude + rotateToAngleRate;
      double rightStickValue = magnitude - rotateToAngleRate;
      myRobot.tankDrive(leftStickValue, rightStickValue);
    } else {
      /* If the turn controller had been enabled, disable it now. */
      if (turnControllerEnabled) {
        turnControllerEnabled = false;
      }
      /* Standard tank drive, no driver assistance. */
      myRobot.tankDrive(stick.getY(Hand.kLeft), stick.getY(Hand.kRight));
    }
    
  }

}
