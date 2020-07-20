/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.DigitalOutput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.unit_conversion.Angle;
import edu.wpi.first.wpilibj.Servo;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final DifferentialDrive m_robotDrive
      = new DifferentialDrive(new PWMVictorSPX(2), new PWMVictorSPX(1));
  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();
  private final PWMSpeedController m_motor = new Victor(0);
  private final DigitalOutput LeftMotorReverse = new DigitalOutput(8);
  private final DigitalOutput RightMotorReverse = new DigitalOutput(9);
  private double mNumber = 5;
  private Angle angle = new Angle();
  private Servo servo = new Servo(4);

  


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //SmartDashboard.putNumber("Angle", mNumber);
    angle.setOldRange(-1,1);
    angle.setNewRange(0, 180);
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
    SmartDashboard.putNumber("Angle", servo.getAngle());;
    
    // Drive for 2 seconds
   if (m_timer.get() < 2.0) {
      m_robotDrive.arcadeDrive(1, 0.0); // drive forwards half speed
      LeftMotorReverse.set(false);
      RightMotorReverse.set(false);
    } else if (m_timer.get() < 4.0){
      m_robotDrive.arcadeDrive(1, 0.0); // Drive Backwards @ half speed
      LeftMotorReverse.set(true);
      RightMotorReverse.set(true);
    }
    else {
      m_robotDrive.stopMotor(); // stop robot
      LeftMotorReverse.set(false);
      RightMotorReverse.set(false);
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
    angle.setInputNumber(m_stick.getRawAxis(0));
    angle.convert();
    servo.setAngle(angle.getOutputNumber());
    SmartDashboard.putNumber("Angle", angle.getOutputNumber());
    if (m_stick.getY() < -0.5) {
      m_robotDrive.arcadeDrive(Math.abs(Math.pow(m_stick.getY(), 2)), 0.0); // drive forwards half speed
      LeftMotorReverse.set(false);
      RightMotorReverse.set(false);
    } else if( m_stick.getY() > 0.5){
      m_robotDrive.arcadeDrive(Math.abs(Math.pow(m_stick.getY(), 2)), 0.0); // Drive Backwards @ half speed
      LeftMotorReverse.set(true);
      RightMotorReverse.set(true);
    } else{
    m_robotDrive.stopMotor(); // stop robot
    LeftMotorReverse.set(false);
    RightMotorReverse.set(false);
    }  
    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
