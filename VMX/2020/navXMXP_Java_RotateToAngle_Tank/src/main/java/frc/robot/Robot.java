/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//Import Drive Code
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//import encoder
import edu.wpi.first.wpilibj.Encoder;

//import PID
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpiutil.math.MathUtil;


//TODO Add NavX turn code

//TODO Tune PID
//TODO Clean Up Code

/**
 * This is a demo program showing the use of the navX MXP to implement
 * the "rotate to angle", "zero yaw" and "drive straight" on a Tank
 * drive system.
 *
 * If Left Joystick Button 0 is pressed, a "turn" PID controller will 
 * set to point to a target angle, and while the button is held the drive
 * system will rotate to that angle (NOTE:  tank drive systems cannot simultaneously
 * move forward/reverse while rotating).
 *
 * This example also includes a feature allowing the driver to "reset"
 * the "yaw" angle.  When the reset occurs, the new gyro angle will be
 * 0 degrees.  This can be useful in cases when the gyro drifts, which
 * doesn't typically happen during a FRC match, but can occur during
 * long practice sessions.
 *
 * Finally, if Left Joystick button 2 is held, the "turn" PID controller will
 * be set to point to the current heading, and while the button is held,
 * the driver system will continue to point in the direction.  The robot 
 * can drive forward and backward (the magnitude of motion is the average
 * of the Y axis values on the left and right joysticks).
 *
 * Note that the PID Controller coefficients defined below will need to
 * be tuned for your drive system.
 */

public class Robot extends TimedRobot {

  AHRS ahrs;
  //Setup Constants
  private static final int leftMotorPort = 0;
  private static final int rightMotorPort = 1;
  private static final int leftEncPortA =  0;
  private static final int leftEncPortB = 1;
  private static final int rightEncPortA = 2;
  private static final int rightEncPortB = 3;
  private static final int joystickPort = 0;

  //PID Gains
  double Kp = .09;
  double Ki = 0.001;
  double Kd = .004;
  double desiredSpeed = 100;
  double pidOutput;
  //NavX Specific Constants
  static final double kToleranceDegrees = 2.0f;    
    
  static final double kTargetAngleDegrees = 90.0f;

  //Instantiate the motors and encoders
  private final PWMVictorSPX m_leftMotor = new PWMVictorSPX(leftMotorPort);
  private final PWMVictorSPX m_rightMotor = new PWMVictorSPX(rightMotorPort);
  private final DifferentialDrive myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final XboxController stick = new XboxController(joystickPort);

  private final Encoder leftEncoder = new Encoder(leftEncPortA, leftEncPortB);
  private final Encoder rightEncoder = new Encoder(rightEncPortA, rightEncPortB);
  
  //Instantiate the dashboard variables
  private double leftSpeed;
  private double rightSpeed;
  private double leftDistance;
  private double rightDistance;

  //Instantiate the PIDController
  PIDController my_pid = new PIDController(Kp, Ki, Kd);


  

  @Override
  public void robotInit() {
    
   // PIDController my_pid = new PIDController(Kp, Ki, Kd);
   SmartDashboard.putNumber("P", Kp);
   SmartDashboard.putNumber("I", Ki);
   SmartDashboard.putNumber("D", Kd);
   SmartDashboard.putNumber("Setpoint", desiredSpeed);
   myRobot.setExpiration(0.1);

    //Initialize the NavX
      try {
          /***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
      ahrs = new AHRS(SPI.Port.kMXP); 
      } catch (RuntimeException ex ) {
          DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
      }
      //They instantiated the turn controller like this...
      /*turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
      turnController.setInputRange(-180.0f, 180.0f);
      turnController.setOutputRange(-1.0, 1.0);
      turnController.setAbsoluteTolerance(kToleranceDegrees);
      turnController.setContinuous(true);
      turnController.disable();*/
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
    rightEncoder.reset();
    leftEncoder.reset();
   
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
        //double rightSpeed =  rightEncoder.getRate();
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    pidOutput = my_pid.calculate(leftEncoder.getDistance(), desiredSpeed);
    pidOutput = MathUtil.clamp(-pidOutput, -.6, .6);

    myRobot.arcadeDrive(pidOutput, 0);
    
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
    Timer.delay(0.020);		/* wait for one motor update time period (50Hz)     */
          
          boolean zero_yaw_pressed = stick.getBButton();
          if ( zero_yaw_pressed ) {
              ahrs.zeroYaw();
          }

          /* Display 6-axis Processed Angle Data                                      */
          SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
          SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
          SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
          SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
          SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
          
          /* Display tilt-corrected, Magnetometer-based heading (requires             */
          /* magnetometer calibration to be useful)                                   */
          
          SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());
          
          /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
          SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

          /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
          /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP            */
          
          SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
          SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());

          /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
          
          SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
          SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
          SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
          SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());

          /* Display estimates of velocity/displacement.  Note that these values are  */
          /* not expected to be accurate enough for estimating robot position on a    */
          /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
          /* of these errors due to single (velocity) integration and especially      */
          /* double (displacement) integration.                                       */
          
          SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX());
          SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY());
          SmartDashboard.putNumber(   "Displacement_X",       ahrs.getDisplacementX());
          SmartDashboard.putNumber(   "Displacement_Y",       ahrs.getDisplacementY());
          
          /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
          /* NOTE:  These values are not normally necessary, but are made available   */
          /* for advanced users.  Before using this data, please consider whether     */
          /* the processed data (see above) will suit your needs.                     */
          
          SmartDashboard.putNumber(   "RawGyro_X",            ahrs.getRawGyroX());
          SmartDashboard.putNumber(   "RawGyro_Y",            ahrs.getRawGyroY());
          SmartDashboard.putNumber(   "RawGyro_Z",            ahrs.getRawGyroZ());
          SmartDashboard.putNumber(   "RawAccel_X",           ahrs.getRawAccelX());
          SmartDashboard.putNumber(   "RawAccel_Y",           ahrs.getRawAccelY());
          SmartDashboard.putNumber(   "RawAccel_Z",           ahrs.getRawAccelZ());
          SmartDashboard.putNumber(   "RawMag_X",             ahrs.getRawMagX());
          SmartDashboard.putNumber(   "RawMag_Y",             ahrs.getRawMagY());
          SmartDashboard.putNumber(   "RawMag_Z",             ahrs.getRawMagZ());
          SmartDashboard.putNumber(   "IMU_Temp_C",           ahrs.getTempC());
          
          /* Omnimount Yaw Axis Information                                           */
          /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
          AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
          SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
          SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
          
          /* Sensor Board Information                                                 */
          SmartDashboard.putString(   "FirmwareVersion",      ahrs.getFirmwareVersion());
          
          /* Quaternion Data                                                          */
          /* Quaternions are fascinating, and are the most compact representation of  */
          /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
          /* from the Quaternions.  If interested in motion processing, knowledge of  */
          /* Quaternions is highly recommended.                                       */
          SmartDashboard.putNumber(   "QuaternionW",          ahrs.getQuaternionW());
          SmartDashboard.putNumber(   "QuaternionX",          ahrs.getQuaternionX());
          SmartDashboard.putNumber(   "QuaternionY",          ahrs.getQuaternionY());
          SmartDashboard.putNumber(   "QuaternionZ",          ahrs.getQuaternionZ());
          
          /* Connectivity Debugging Support                                           */
          SmartDashboard.putNumber(   "IMU_Byte_Count",       ahrs.getByteCount());
          SmartDashboard.putNumber(   "IMU_Update_Count",     ahrs.getUpdateCount());
          
          //drive code
          myRobot.tankDrive(stick.getY(Hand.kLeft), stick.getY(Hand.kRight));
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
    if (leftStick.getRawButton(0)) {
      /*
       * While this button is held down, rotate to target angle. Since a Tank drive
       * system cannot move forward simultaneously while rotating, all joystick input
       * is ignored until this button is released.
       */
      if (!turnController.isEnabled()) {
        turnController.setSetpoint(kTargetAngleDegrees);
        rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
        turnController.enable();
      }
      double leftStickValue = rotateToAngleRate;
      double rightStickValue = rotateToAngleRate;
      myRobot.tankDrive(leftStickValue, rightStickValue);
    } else if (leftStick.getRawButton(1)) {
      /*
       * "Zero" the yaw (whatever direction the sensor is pointing now will become the
       * new "Zero" degrees.
       */
      ahrs.zeroYaw();
    } else if (leftStick.getRawButton(2)) {
      /*
       * While this button is held down, the robot is in "drive straight" mode.
       * Whatever direction the robot was heading when "drive straight" mode was
       * entered will be maintained. The average speed of both joysticks is the
       * magnitude of motion.
       */
      if (!turnController.isEnabled()) {
        // Acquire current yaw angle, using this as the target angle.
        turnController.setSetpoint(ahrs.getYaw());
        rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
        turnController.enable();
      }
      double magnitude = (leftStick.getY() + rightStick.getY()) / 2;
      double leftStickValue = magnitude + rotateToAngleRate;
      double rightStickValue = magnitude - rotateToAngleRate;
      myRobot.tankDrive(leftStickValue, rightStickValue);
    } else {
      /* If the turn controller had been enabled, disable it now. */
      if (turnController.isEnabled()) {
        turnController.disable();
      }
      /* Standard tank drive, no driver assistance. */
      myRobot.tankDrive(leftStick.getY(), rightStick.getY());
    }
  }

  @Override
  /* This function is invoked periodically by the PID Controller, */
  /* based upon navX MXP yaw angle input and PID Coefficients. */
  public void pidWrite(double output) {
    rotateToAngleRate = output;
  }
}
