/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;

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

  //robot map
  private static final int frontLeftMotorPort = 0;
  private static final int rearLeftMotorPort = 2;
  private static final int rearRightMotorPort = 3;
  private static final int frontRightMotorPort = 1;


  private static final int JoystickPort = 0;

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
  
  private TitanQuad frontLeft = new TitanQuad(42,15600, frontLeftMotorPort);
  private TitanQuad rearLeft = new TitanQuad(42,15600, rearLeftMotorPort);
  private TitanQuad rearRight = new TitanQuad(42,15600, rearRightMotorPort);
  private TitanQuad frontRight = new TitanQuad(42,15600, frontRightMotorPort);
  
  private final TitanQuadEncoder frontLeftEnc =
  new TitanQuadEncoder(frontLeft, frontLeftMotorPort, (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1));
  private TitanQuadEncoder rearLeftEnc = new TitanQuadEncoder(rearLeft, rearLeftMotorPort, (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1));
  private final TitanQuadEncoder m_encoder =
  new TitanQuadEncoder(rearRight, rearRightMotorPort, (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1));
  private final TitanQuadEncoder frontRightEnc =
  new TitanQuadEncoder(frontRight, frontRightMotorPort, (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1));

  
  private Joystick stick = new Joystick(JoystickPort);
  
  @Override
  public void robotInit() {

  }

  @Override
  public void teleopPeriodic() {
    frontLeft.set(stick.getY());
    rearLeft.set(stick.getY());
    rearRight.set(stick.getY());
    frontRight.set(stick.getY());
    SmartDashboard.putNumber("frontLeft Encoder Distance", frontLeftEnc.getEncoderDistance());
    SmartDashboard.putNumber("frontLeft Encoder Rate",frontLeftEnc.getRPM());
    SmartDashboard.putNumber("Left Encoder Distance", rearLeftEnc.getEncoderDistance());
    SmartDashboard.putNumber("Left Encoder Rate",rearLeftEnc.getRPM());
    SmartDashboard.putNumber("Encoder Distance", m_encoder.getEncoderDistance());
    SmartDashboard.putNumber("Encoder Rate", m_encoder.getSpeed());
    SmartDashboard.putNumber("frontRight Encoder Distance", frontRightEnc.getEncoderDistance());
    SmartDashboard.putNumber("frontRight Encoder Rate",frontRightEnc.getRPM());
  }
}
