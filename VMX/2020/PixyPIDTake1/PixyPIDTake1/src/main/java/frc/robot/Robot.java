/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

//For Pixy
import java.util.ArrayList;
import io.github.pseudoresonance.pixy2api.*;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.I2CLink;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  // Create constants for Pixy.
  private Pixy2 pixycam;
  boolean isCamera = false ;
  //private SPILink spi;
  int state=- 1 ;


  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    //Instantiate Pixy Object
    pixycam = Pixy2.createInstance(new I2CLink());
    
    // Creates a PIDController with gains kP(0), kI(0), and kD(0)
    //PIDController pid = new PIDController(0, 0,0); 

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
    if (!isCamera)
      state = pixycam.init( ); // if no camera present, try to initialize
      isCamera = state>= 0 ;

    //adjust Pixy Lamp
    pixycam.setLamp((byte) 1, (byte) 1);
    


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
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
       // System.out.println("Auto selected: " + m_autoSelected);

        //Pixy Block Begin...
          
          
          SmartDashboard.putBoolean( "Camera" , isCamera); //publish if we are connected
          pixycam.getCCC().getBlocks( false , 255 , 255 ); //run getBlocks with arguments to have the camera
          //acquire target data
          ArrayList<Block> blocks = pixycam.getCCC().getBlockCache(); //assign the data to an ArrayList for convinience
          if (blocks.size() > 0 )
            {
              double xcoord = blocks.get( 0 ).getX(); // x position of the largest target
              double ycoord = blocks.get( 0 ).getY(); // y position of the largest target
              String data = blocks.get( 0 ).toString(); // string containing target info
              //servos
              int panOffset = pixycam.getFrameWidth()/2 - (int) xcoord;//get the x offset
              //panOffset = (int)panOffset/1000;
              panOffset = MathUtil.clamp((int) panOffset,0,1000);
              //pixycam.setServos((byte) panOffset, (byte) 500);
              //hopefully this works for the pan servo.
              SmartDashboard.putBoolean( "present" , true ); // show there is a target present
              SmartDashboard.putNumber( "Xccord" ,xcoord);
              SmartDashboard.putNumber( "Ycoord" , ycoord);
              SmartDashboard.putString( "Data" , data );
              SmartDashboard.putNumber( "size" , blocks.size());//push to dashboard how many targets are detected
            }
        else
          SmartDashboard.putBoolean( "present" , false );
        //End Pixy Section
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
  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledInit() {
    if (isCamera){
      pixycam.setLamp((byte) 0, (byte) 0);
      
    }
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
