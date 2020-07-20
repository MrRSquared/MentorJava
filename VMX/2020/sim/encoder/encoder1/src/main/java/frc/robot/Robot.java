/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpiutil.math.MathUtil;
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

  private static final int deviceID = 1;
  private WPI_TalonFX m_left_motor;
  private WPI_TalonFX m_right_motor;
  //private double  m_encoder;
  private CANEncoder m_right_encoder;
  private double xValue = 1;
  private double yValue = 0;
  private double m_encoder = xValue*3.0;

  double Kp = .01;
  double Ki = .01;
  double Kd = .01;
  double desiredDistance = 10;
  double pidOutput;
      

      private DifferentialDrive m_myRobot;

      PIDController pid = new PIDController(Kp, Ki, Kd);
      

      private ShuffleboardTab tab = Shuffleboard.getTab("Drive");
      private NetworkTableEntry maxSpeed =
          tab.add("Max Speed", 1)
             .getEntry();

     

    

  @Override
  public void robotInit() {

    m_left_motor = new WPI_TalonFX(1);
    m_right_motor = new WPI_TalonFX(2);
    m_myRobot = new DifferentialDrive(m_left_motor,
    m_right_motor);
    SmartDashboard.putNumber("P", Kp);
    SmartDashboard.putNumber("I", Ki);
    SmartDashboard.putNumber("D", Kd);
    SmartDashboard.putNumber("Setpoint", desiredDistance);



   
   // m_right_encoder = m_right_motor.getEncoder();


    /*
     * Defines how far the mechanism attached to the encoder moves per pulse. In
     * this case, we assume that a 360 count encoder is directly
     * attached to a 3 inch diameter (1.5inch radius) wheel,
     * and that we want to measure distance in inches.
     */
    //m_encoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);

    /*
     * Defines the lowest rate at which the encoder will
     * not be considered stopped, for the purposes of
     * the GetStopped() method. Units are in distance / second,
     * where distance refers to the units of distance
     * that you are using, in this case inches.
     */
    //m_encoder.setMinRate(1.0);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Encoder Distance", m_encoder);
    SmartDashboard.putNumber("pid Output", pidOutput);
    Kp =SmartDashboard.getNumber("P", Kp);
    Ki = SmartDashboard.getNumber("I", Ki);
    Kd = SmartDashboard.getNumber("D", Kd);
    desiredDistance = SmartDashboard.getNumber("Setpoint", desiredDistance);
    pid.setP(Kp);
    pid.setI(Ki);
    pid.setD(Kd);
    SmartDashboard.putNumber("NewKP", Kp);

  }

  @Override
  public void teleopInit() {

    m_encoder = 3;

  }

  @Override
  public void teleopPeriodic() {
   
   pidOutput = pid.calculate(m_encoder, desiredDistance);
   //MathUtil.clamp(pidOutput, -0.5, 0.5);

  //if (m_encoder<= desiredDistance){
    if (m_left_motor.get() > 0){
    m_encoder += .75;
    } else if (m_left_motor.get()<0) {
      m_encoder -= .75;
    }
       SmartDashboard.putNumber("Pid Output",pidOutput);
       m_myRobot.arcadeDrive(pidOutput, yValue);

  //} else{

  //  m_myRobot.arcadeDrive(0, 0);

  }

   
   


  }

