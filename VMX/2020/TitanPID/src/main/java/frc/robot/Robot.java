/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  //robot map
  private static final int frontLeftMotorPort = 0;
  private static final int rearLeftMotorPort = 2;
  private static final int rearRightMotorPort = 3;
  private static final int frontRightMotorPort = 1;


  private static final int JoystickPort = 0;

  double kP = 0.09;
  double kI = 0.001;
  double kD = 0.004;
  double desiredDistance= 10;
  double pidOutput;
  double minSpeed = -1;
  double maxSpeed = 1;
  double encoderDistance;
  double leftSpeed;

  private TitanQuad frontLeft = new TitanQuad(42,15600, frontLeftMotorPort);
  private TitanQuad rearLeft = new TitanQuad(42,15600, rearLeftMotorPort);
  private TitanQuad rearRight = new TitanQuad(42,15600, rearRightMotorPort);
  private TitanQuad frontRight = new TitanQuad(42,15600, frontRightMotorPort);

  private Encoder leftEncoder = new Encoder(0, 1);
  
  
  private final TitanQuadEncoder m_encoder =
  new TitanQuadEncoder(rearRight, rearRightMotorPort, (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1));
  
  private Joystick stick = new Joystick(JoystickPort);

  PIDController my_pid = new PIDController(kP, kI, kD);

  private MecanumDrive m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer.getInstance().startAutomaticCapture();
    frontRight.setInverted(true);
    SmartDashboard.putNumber("Setpoint", desiredDistance);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    desiredDistance = SmartDashboard.getNumber("Setpoint", desiredDistance);
    encoderDistance=  m_encoder.getEncoderDistance();
    leftSpeed = leftEncoder.getRate();
    SmartDashboard.putNumber("Encoder Distance", encoderDistance);
    SmartDashboard.putNumber("left encoder", leftSpeed);
    
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    frontLeft.set(stick.getY());
    rearLeft.set(stick.getY());
    rearRight.set(stick.getY());
    frontRight.set(stick.getY());
    //SmartDashboard.putNumber("frontLeft Encoder Distance", frontLeftEnc.getEncoderDistance());
    //SmartDashboard.putNumber("frontLeft Encoder Rate",frontLeftEnc.getRPM());
    //SmartDashboard.putNumber("Left Encoder Distance", rearLeftEnc.getEncoderDistance());
    //SmartDashboard.putNumber("Left Encoder Rate",rearLeftEnc.getRPM());
    SmartDashboard.putNumber("Encoder Distance", m_encoder.getEncoderDistance());
    
    //SmartDashboard.putNumber("Encoder Rate", m_encoder.getSpeed());
    //SmartDashboard.putNumber("frontRight Encoder Distance", frontRightEnc.getEncoderDistance());
    //SmartDashboard.putNumber("frontRight Encoder Rate",frontRightEnc.getRPM());    
  }

  /**
   * This function is called once when teleop is enabled.
   */
  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.driveCartesian(stick.getRawAxis(1), stick.getRawAxis(4), stick.getRawAxis(3));
    
  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  /**
   * This function is called periodically when disabled.
   */
  @Override
  public void disabledPeriodic() {
  }

  /**
   * This function is called once when test mode is enabled.
   */
  @Override
  public void testInit() {
    m_encoder.reset();

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    pidOutput = my_pid.calculate(encoderDistance, desiredDistance);
    pidOutput = MathUtil.clamp(-pidOutput, -.6, .6);
    m_robotDrive.driveCartesian(pidOutput, stick.getRawAxis(4), stick.getRawAxis(3));
    
  }
}
