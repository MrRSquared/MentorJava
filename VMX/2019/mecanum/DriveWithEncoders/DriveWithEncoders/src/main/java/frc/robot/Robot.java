/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and speed controller inputs also
 * range from -1 to 1 making it easy to work together.
 *
 * <p>In addition, the encoder value of an encoder connected to ports 0 and 1 is
 * consistently sent to the Dashboard.
 */
public class Robot extends TimedRobot {
  private static final int kLeftRearMotorPort = 0;
  private static final int kRightRearMotorPort = 1;
  private static final int kLeftFrontMotorPort = 2;
  private static final int kRightFrontMotorPort = 3;
  private static final int kJoystickPort = 0;
  private static final int kleftEncoderPortA = 8;
  private static final int kleftEncoderPortB = 9;
  private static final int krightEncoderPortA = 0;
  private static final int krightEncoderPortB = 1;

  private SpeedController m_leftRearMotor;
  private SpeedController m_rightRearMotor;
  private SpeedController m_leftFrontMotor;
  private SpeedController m_rightFrontMotor;

  private Joystick m_joystick;
  private Encoder m_leftEncoder;
  private Encoder m_rightEncoder;
  private MecanumDrive m_driveTrain;
  private double m_turn;

  @Override
  public void robotInit() {
    m_leftRearMotor = new PWMVictorSPX(kLeftRearMotorPort);
    m_rightRearMotor = new PWMVictorSPX(kRightRearMotorPort);
    m_leftFrontMotor = new PWMVictorSPX(kLeftFrontMotorPort);
    m_rightFrontMotor = new PWMVictorSPX(kRightFrontMotorPort);
    m_joystick = new Joystick(kJoystickPort);
    m_leftEncoder = new Encoder(kleftEncoderPortA, kleftEncoderPortB);
    m_rightEncoder = new Encoder(krightEncoderPortA, krightEncoderPortB);
    m_driveTrain = new MecanumDrive(m_leftFrontMotor, m_leftRearMotor, m_rightFrontMotor, m_rightRearMotor);
    // Use SetDistancePerPulse to set the multiplier for GetDistance
    // This is set up assuming a 6 inch wheel with a 360 CPR encoder.
    m_leftEncoder.setDistancePerPulse((Math.PI * 6) / 360.0);
    m_rightEncoder.setDistancePerPulse((Math.PI * 6) / 360.0);
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Left Encoder", m_leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Encoder", m_rightEncoder.getDistance());
  }
  @Override
  public void autonomousPeriodic() {
    m_driveTrain.driveCartesian(0, 0, 0);
    if(m_joystick.getPOV()==315){
      m_leftFrontMotor.set(.5);
    } else if (m_joystick.getPOV()==45){
      m_rightFrontMotor.set(.5);
    }else if (m_joystick.getPOV()==135){
      m_rightRearMotor.set(.5);
    }else if (m_joystick.getPOV()==225){
      m_leftRearMotor.set(.5);
    } 
  }

  @Override
  public void teleopPeriodic() {
    if (m_joystick.getRawAxis(4)>=.5 || m_joystick.getRawAxis(4)<=.5){
      m_turn = m_joystick.getRawAxis(4);
    }else{
      m_turn = 0;
    }

    m_driveTrain.driveCartesian(m_joystick.getY(Hand.kLeft),m_joystick.getX(Hand.kLeft),m_turn);
  }
}
