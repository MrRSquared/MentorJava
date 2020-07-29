/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //Constants
  //left motors
  private static final int frontLeftMotorPort = 0;
  private static final int rearLeftMotorPort = 2;
  
  //right motors
  private static final int frontRightMotorPort = 1;
  private static final int rearRightMotorPort = 3;
  
  //Stick
  private static final int JoystickPort = 0;
  //Titan
  private static final int titanID = 42;

  private static int wheelDiameter = 80;
  private static int cpr = 150;

  //Gyro
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
 
  //Instantiate the motors
  private TitanQuad frontLeft = new TitanQuad(titanID,15600, frontLeftMotorPort);
  private TitanQuad rearLeft = new TitanQuad(titanID,15600, rearLeftMotorPort);
  private TitanQuad frontRight = new TitanQuad(titanID,15600, frontRightMotorPort);
  private TitanQuad rearRight = new TitanQuad(titanID,15600, rearRightMotorPort);

  //Encoders
  
  private final TitanQuadEncoder frontLeftEnc =
  new TitanQuadEncoder(frontLeft, frontLeftMotorPort, (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1));
  private TitanQuadEncoder rearLeftEnc = new TitanQuadEncoder(rearLeft, rearLeftMotorPort, (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1));
  private final TitanQuadEncoder rearRightEnc =
  new TitanQuadEncoder(rearRight, rearRightMotorPort, (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1));
  private final TitanQuadEncoder frontRightEnc =
  new TitanQuadEncoder(frontRight, frontRightMotorPort, (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1), (3.14 * 2 * 2) / (1120 * 1/1));

  private MecanumDrive robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
  private Joystick stick = new Joystick(JoystickPort);

  //much of what follows before the overridden class is default logic.
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
 
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    rearRight.setInverted(true);
    //rearLeft.setInverted(true);
    robotDrive.setRightSideInverted(true);
    
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

    //SmartDashboard.putNumber("fL Enc", frontLeftEnc.getEncoderDistance());
    //SmartDashboard.putNumber("rL Enc", rearLeftEnc.getEncoderDistance());
    //SmartDashboard.putNumber("fR Enc", frontRightEnc.getEncoderDistance());
    //SmartDashboard.putNumber("rr dis.", rearRightEnc.getEncoderDistance());
    //SmartDashboard.putNumber("rr speed.", rearRightEnc.getRPM());
    
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
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
    robotDrive.driveCartesian(stick.getX(Hand.kLeft), stick.getY(Hand.kRight), stick.getRawAxis(3),m_gyro.getYaw());
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
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
