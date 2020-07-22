/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//TODO: Tune PID to stop on a dime


import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
 */
public class Robot extends TimedRobot {
  //Setup Constants
  private static final int leftMotorPort = 0;
  private static final int rightMotorPort = 1;
  private static final int leftEncPortA =  0;
  private static final int leftEncPortB = 1;
  private static final int rightEncPortA = 2;
  private static final int rightEncPortB = 3;

  //PID Constants

  double Kp = .09;
  double Ki = 0.001;
  double Kd = .004;
  double desiredSpeed = 100;
  double pidOutput;
    

  private final PWMVictorSPX m_leftMotor = new PWMVictorSPX(leftMotorPort);
  private final PWMVictorSPX m_rightMotor = new PWMVictorSPX(rightMotorPort);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  //private final Joystick m_stick = new Joystick(joystickPort);
  

  private final Encoder leftEncoder = new Encoder(leftEncPortA, leftEncPortB);
  private final Encoder rightEncoder = new Encoder(rightEncPortA, rightEncPortB);
  private double leftSpeed;
  private double rightSpeed;
  private double leftDistance;
  private double rightDistance;
  PIDController my_pid = new PIDController(Kp, Ki, Kd);
  
  @Override
  public void robotInit() {

   // PIDController my_pid = new PIDController(Kp, Ki, Kd);
    SmartDashboard.putNumber("P", Kp);
    SmartDashboard.putNumber("I", Ki);
    SmartDashboard.putNumber("D", Kd);
    SmartDashboard.putNumber("Setpoint", desiredSpeed);

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
    Kp =SmartDashboard.getNumber("P", Kp);
    Ki = SmartDashboard.getNumber("I", Ki);
    Kd = SmartDashboard.getNumber("D", Kd);
    my_pid.setP(Kp);
    my_pid.setI(Ki);
    my_pid.setD(Kd);
    SmartDashboard.putNumber("NewKP", Kp);
  }

  @Override
  public void autonomousInit() {
    rightEncoder.reset();
    leftEncoder.reset();
  }

  @Override
  public void autonomousPeriodic() {
    //double rightSpeed =  rightEncoder.getRate();
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    pidOutput = my_pid.calculate(leftEncoder.getDistance(), desiredSpeed);
    pidOutput = MathUtil.clamp(-pidOutput, -.6, .6);

    m_robotDrive.arcadeDrive(pidOutput, 0);
  }
}
