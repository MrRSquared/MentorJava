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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput; 
import edu.wpi.first.wpilibj.Ultrasonic;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final DifferentialDrive m_robotDrive
      = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();
  private final AnalogInput m_ultrasonic = new AnalogInput(0);
  DigitalOutput leftMotorReverse;   
  DigitalOutput rightMotorReverse; 
  // Creates a ping-response Ultrasonic object on DIO 1 and 2.
  Ultrasonic ultrasonic = new Ultrasonic(11, 10);
  


  final static int leftReverseChannel = 8; 
  final static int rightReverseChannel = 9;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    leftMotorReverse = new DigitalOutput(leftReverseChannel);  
    rightMotorReverse = new DigitalOutput(rightReverseChannel);

    // Start the ultrasonic in automatic mode
    ultrasonic.setAutomaticMode(false);
  }

  @ Override
  public void robotPeriodic(){

   

    double m_distance = (ultrasonic.getRangeInches()* 100 * 343.0 /100000000);
    SmartDashboard.putBoolean("Does it work?", ultrasonic.isRangeValid());
    
    SmartDashboard.putNumber("My DIstance", m_distance);
    
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
    // Drive for 2 
    double myDistance = (m_ultrasonic.getVoltage());
    SmartDashboard.putNumber("My Distance", myDistance);
    /**if (m_stick.getY() < -0.5){
      leftMotorReverse.set(false);
      rightMotorReverse.set(false);
      m_robotDrive.arcadeDrive(m_stick.getY(), 0);
    } else if (m_stick.getY() > 0.5) {
      leftMotorReverse.set(true);
      rightMotorReverse.set(true);
      m_robotDrive.arcadeDrive(m_stick.getY(), 0);
    } else {*/
      m_robotDrive.stopMotor(); // stop robot
    //}
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
    m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
