/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Sample program displaying the value of a quadrature encoder on the SmartDashboard. Quadrature
 * Encoders are digital sensors which can detect the amount the encoder has rotated since starting
 * as well as the direction in which the encoder shaft is rotating. However, encoders can not tell
 * you the absolute position of the encoder shaft (ie, it considers where it starts to be the zero
 * position, no matter where it starts), and so can only tell you how much the encoder has rotated
 * since starting. Depending on the precision of an encoder, it will have fewer or greater ticks per
 * revolution; the number of ticks per revolution will affect the conversion between ticks and
 * distance, as specified by DistancePerPulse. One of the most common uses of encoders is in the
 * drivetrain, so that the distance that the robot drives can be precisely controlled during the
 * autonomous mode.
 */
public class Robot extends TimedRobot {
  //Constants (eventually, this will be in RobotMap)

  private static final int kleftMotorPort = 0;
  private static final int krightMotorPort = 1;

  private static final int kleftEncoderPortA = 2;
  private static final int kleftEncoderPortB = 3;
  private static final int krighEncoderPortA = 0;
  private static final int krightEncoderPortB = 1;
  private static final int kJoystickPort = 0;

  /**
   * The Encoder object is constructed with 4 parameters, the last two being optional. The first two
   * parameters (1, 2 in this case) refer to the ports on the roboRIO which the encoder uses.
   * Because a quadrature encoder has two signal wires, the signal from two DIO ports on the roboRIO
   * are used. The third (optional)  parameter is a boolean which defaults to false. If you set this
   *  parameter to true, the direction of the encoder will be reversed, in case it makes more sense
   * mechanically. The final (optional) parameter specifies encoding rate (k1X, k2X, or k4X) and
   * defaults to k4X. Faster (k4X) encoding gives greater positional precision but more noise in the
   * rate.
   */
  private final Encoder m_leftEncoder =
      new Encoder(kleftEncoderPortA, kleftEncoderPortB, false, CounterBase.EncodingType.k4X);
  
  private final Encoder m_rightEncoder =
      new Encoder(krighEncoderPortA, krightEncoderPortB, false, CounterBase.EncodingType.k4X);
  
  //Drive motors
  private SpeedController m_leftMotor;
  private SpeedController m_rightMotor;

  //Drivetrain
  private DifferentialDrive m_driveTrain;

  //Joystick
  private Joystick m_joystick;


  @Override
  public void robotInit() {
    m_leftMotor = new PWMVictorSPX(kleftMotorPort);
    m_leftMotor.setInverted(true);
    m_rightMotor = new PWMVictorSPX(krightMotorPort);
    m_rightMotor.setInverted(true);

    m_driveTrain = new DifferentialDrive(m_leftMotor, m_rightMotor);

    m_joystick = new Joystick(kJoystickPort);

    /*
     * Defines the number of samples to average when determining the rate.
     * On a quadrature encoder, values range from 1-255;
     * larger values result in smoother but potentially
     * less accurate rates than lower values.
     */
    m_leftEncoder.setSamplesToAverage(5);
    m_rightEncoder.setSamplesToAverage(5);

    /*
     * Defines how far the mechanism attached to the encoder moves per pulse. In
     * this case, we assume that a 360 count encoder is directly
     * attached to a 3 inch diameter (1.5inch radius) wheel,
     * and that we want to measure distance in inches.
     */
    m_leftEncoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 4);
    m_rightEncoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 4);

    /*
     * Defines the lowest rate at which the encoder will
     * not be considered stopped, for the purposes of
     * the GetStopped() method. Units are in distance / second,
     * where distance refers to the units of distance
     * that you are using, in this case inches.
     */
    m_leftEncoder.setMinRate(1.0);
    m_rightEncoder.setMinRate(1.0);
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Left Encoder Distance", m_leftEncoder.getDistance());
    SmartDashboard.putNumber("Left Encoder Rate", m_leftEncoder.getRate());
    SmartDashboard.putNumber("Right Encoder Distance", m_rightEncoder.getDistance());
    SmartDashboard.putNumber("Right Encoder Rate", m_rightEncoder.getRate());

    m_driveTrain.arcadeDrive(m_joystick.getY(), -m_joystick.getX(), true);
  }
}
