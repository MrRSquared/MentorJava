/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
//import edu.wpi.first.wpilibj.DigitalOutput;   //MOVEME: Rear Wheel Direction
//import frc.robot.AngleConversion;            // MOVEME: Front Wheel Angles
//import edu.wpi.first.wpilibj.Servo;         //  MOVEME: Front Wheel Angles
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;  //We no longer need WPILIB DifferentialDrive, because we modified the class
import frc.robot.CarDrive; //Bring in new Drivetrain for the PiCar

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  //private final DifferentialDrive m_robotDrive
     // = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
  private final CarDrive m_robotDrive 
    = new CarDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));// Use CarDrive Class for PiCar Instead of DifferentialDrive
  
  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();
  //private final AngleConversion m_turnConverter = new AngleConversion();  //MOVEME: Front Wheels
  //private final Servo turnServo = new Servo(3);                          // MOVEME: TODO: Bring input over.
  
  

  //DigitalOutput leftMotorReverse;      //MOVEME: Digital Input for the Rear Wheel Direction
  //DigitalOutput rightMotorReverse;     //MOVEME: Digital Input for the Rear Wheel Direction
  
  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  //final static int leftReverseChannel = 8;  // MOVEME: TODO: Bring input over.
 // final static int rightReverseChannel = 9; // MOVEME: TODO: Bring input over.
  double driveSpeed;  
  
  

  @Override
  public void robotInit() {
    m_robotDrive.stopMotor();  // The Motor Controller for the PiCar likes to start when the VMX starts
    //leftMotorReverse = new DigitalOutput(leftReverseChannel);  //MOVEME: TODO: Bring Input Over
    //rightMotorReverse = new DigitalOutput(rightReverseChannel);//MOVEME: TODO: Bring Input Over
    //m_turnConverter.setOldRange(-1,1);    //MOVEME: Bring Input Over This translates the input from the stick to the 
    //m_turnConverter.setNewRange(27, 153);//MOVEME: TODO: Bring Input Over 
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    /** if (m_timer.get() < 2.0) {
      m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    } else if (m_timer.get() <4.0){
      
      m_robotDrive.arcadeDrive(0.5, 0.0); 
    }*/
    if (m_stick.getY(Hand.kLeft)!=0 || m_stick.getY(Hand.kRight) != 0){
      m_robotDrive.tankDrive(m_stick.getY(Hand.kLeft), m_stick.getY(Hand.kRight));
    }
    else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    driveSpeed = m_stick.getY();
    
    double input = m_stick.getX();
    //m_turnConverter.setInputNumber(input); //MOVEME: 
    //double output = m_turnConverter.getOutputNumber();
    //m_turnConverter.convert();  //MOVEME: TODO: make happen automatically
   // int m_turnOutput = (int) m_turnConverter.getOutputNumber();//MOVEME:

    SmartDashboard.putNumber("input", input);
   // SmartDashboard.putNumber("output",  m_turnOutput);

    
    //turnServo.setAngle(m_turnOutput);//MOVEME:

   /** if (driveSpeed < 0) {
      leftMotorReverse.set(false);//MOVEME:
      rightMotorReverse.set(false);//MOVEME:
    } else {
      leftMotorReverse.set(true);//MOVEME:
      rightMotorReverse.set(true);//MOVEME:
    } */
    //if (driveSpeed >.1 || driveSpeed < -.1){
    m_robotDrive.arcadeDrive(driveSpeed,input);
    //m_robotDrive.stopMotor();
 // } //else{ 
   if (driveSpeed == 0) {m_robotDrive.stopMotor();}
  }
  

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
