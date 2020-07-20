/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.DigitalInput;

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
  private static final int kLeftFrontMotorPort = 3;
  private static final int kRightFrontMotorPort = 2;
  private static final int kJoystickPort = 0;

  //private static final int kleftRearEncoderPortA = 0;
  //private static final int kleftRearEncoderPortB = 1;
  //private static final int krightRearEncoderPortA = 2;
  //private static final int krightRearEncoderPortB = 3;
  private static final int kleftFrontEncoderPortA = 4;
  private static final int kleftFrontEncoderPortB = 5;
  private static final int krightFrontEncoderPortA = 0;
  private static final int krightFrontEncoderPortB = 1;
  

  private SpeedController m_leftRearMotor;
  private SpeedController m_rightRearMotor;
  private SpeedController m_leftFrontMotor;
  private SpeedController m_rightFrontMotor;

  //private DigitalInput m_button;

  //private Encoder m_leftRearEncoder;
  //private Encoder m_rightRearEncoder;
  private Encoder m_leftFrontEncoder;
  private Encoder m_rightFrontEncoder;
  private MecanumDrive m_driveTrain;

  private Joystick m_joystick;

  private double m_turn;

  AHRS ahrs;

  @Override
  public void robotInit() {
    m_leftRearMotor = new PWMVictorSPX(kLeftRearMotorPort);
    m_rightRearMotor = new PWMVictorSPX(kRightRearMotorPort);
    m_leftFrontMotor = new PWMVictorSPX(kLeftFrontMotorPort);
    m_rightFrontMotor = new PWMVictorSPX(kRightFrontMotorPort);
    m_joystick = new Joystick(kJoystickPort);
    //m_leftRearEncoder = new Encoder(kleftRearEncoderPortA, kleftRearEncoderPortB);
    //m_rightRearEncoder = new Encoder(krightRearEncoderPortA, krightRearEncoderPortB);
    m_leftFrontEncoder = new Encoder(kleftFrontEncoderPortA, kleftFrontEncoderPortB);
    m_rightFrontEncoder = new Encoder(krightFrontEncoderPortA, krightFrontEncoderPortB);
    m_driveTrain = new MecanumDrive(m_leftFrontMotor, m_leftRearMotor, m_rightFrontMotor, m_rightRearMotor);
    m_driveTrain.setRightSideInverted(true);
    //Use SetDistancePerPulse to set the multiplier for GetDistance
    // This is set up assuming a 6 inch wheel with a 360 CPR encoder.
    //m_leftRearEncoder.setDistancePerPulse((Math.PI * 6) / 360.0);
    //m_rightRearEncoder.setDistancePerPulse((Math.PI * 6) / 360.0);
    m_leftFrontEncoder.setDistancePerPulse((Math.PI * 6) / 360.0);
    m_rightFrontEncoder.setDistancePerPulse((Math.PI * 6) / 360.0);

    //m_button = new DigitalInput(buttonPort);
    try {
      /***********************************************************************
       * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C) and USB. - See
       * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
       * 
       * navX-Micro: - Communication via I2C (RoboRIO MXP or Onboard) and USB. - See
       * http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
       * 
       * VMX-pi: - Communication via USB. - See
       * https://vmx-pi.kauailabs.com/installation/roborio-installation/
       * 
       * Multiple navX-model devices on a single robot are supported.
       ************************************************************************/
      ahrs = new AHRS(SPI.Port.kMXP);
      // ahrs = new AHRS(SerialPort.Port.kUSB1);
      ahrs.enableLogging(true);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    //SmartDashboard.putNumber("Left Rear Encoder", m_leftRearEncoder.getDistance());
    //SmartDashboard.putNumber("Right Rear Encoder", m_rightRearEncoder.getDistance());
    SmartDashboard.putNumber("Left Front Encoder", m_leftFrontEncoder.getDistance());
    SmartDashboard.putNumber("Right Front Encoder", m_rightFrontEncoder.getDistance());
    /* Display 6-axis Processed Angle Data */
    SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
    SmartDashboard.putBoolean("IMU_IsCalibrating", ahrs.isCalibrating());
    SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
    SmartDashboard.putNumber("IMU_Pitch", ahrs.getPitch());
    SmartDashboard.putNumber("IMU_Roll", ahrs.getRoll());

    //Button
   // SmartDashboard.putBoolean("Button", m_button.get());
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
      m_turn = -m_joystick.getRawAxis(4);
    }else{
      m_turn = 0;
    }

    m_driveTrain.driveCartesian(-m_joystick.getX(Hand.kLeft),m_joystick.getY(Hand.kLeft),m_turn);
  }
}
