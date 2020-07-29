/*----------------------------------------------------------------------------*
/
/* Copyright (c) 2018 FIRST. All Rights Reserved.
*/
/* Open Source Software - may be modified and shared by FRC teams. The code
*/
/* must be accompanied by the FIRST BSD license file in the root directory of
*/
/* the project.
*/
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
import io.github.pseudoresonance.pixy2api.*;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.I2CLink;
//Import our custom PID class from our robot.
import frc.robot.MyPID;
//import our conversion class

/**
 * PIXY SPI EXAMPLE A few notes 1.) Two things need to be added to the
 * build.gradle file. 
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  private Pixy2 pixycam;
  boolean isCamera = false;
  int state = -1;
  // Begin PID piece for Pixy
  private int panDesiredPosition = 500; // this is the default position (Pixy calls it m_command so as to use it for
                                        // velocity too)
  private int tiltDesiredPosition = 500;
  private int panOffset;
  private int tiltOffset;

  private MyPID panPID;
  private MyPID tiltPID;

  // Drive logic for following the target.

  // PID Constants
  private int panP = 400;
  private int panI = 0;
  private int panD = 600;

  private int tiltP = 500;
  private int tiltI = 0;
  private int tiltD = 700;

  // Drive PID setup
  private PIDController drivePIDX;
  private double driveXP = -.2;
  private double driveXI = 0;
  private double driveXD = 0;

  //private DifferentialDrive m_drive;
  private boolean followToggle;

  Victor m_left = new Victor(0);
  Victor m_right = new Victor(1);
  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
  private double leftSpeed = 0;
  private double rightSpeed = 0;
  private Conversion convertDriveSpeed = new Conversion(500, 1000, 0, 1, -1);

  //Joystick
  private Joystick stick = new Joystick(0);

  double minCommand = .35;

@Override
public void robotInit () {

 pixycam = Pixy2.createInstance(new I2CLink()); // Creates a new Pixy2 camera using SPILink
 pixycam.init(); // Initializes the camera and prepares to send/receive data
 panPID = new MyPID();
 panPID.setPIDs(panP, panI, panD);
 tiltPID = new MyPID();
 tiltPID.setPIDs(tiltP, tiltI, tiltD);
 //SmartDashboard.putNumber("DriveXP", driveXP);
  
  //Drivetrain logic
  //PID
  // Creates a PIDController with gains kP, kI, and kD
  //drivePIDX = new PIDController(driveXP,driveXI, driveXD);
  SmartDashboard.putNumber("MinCommand",0);
  
}

@Override
  public void disabledInit() {

   // pixycam.setLamp((byte) 0, (byte) 0);
    
  }

@Override
public void robotPeriodic() {
    //Get the smartdashboard toggle.
    followToggle = SmartDashboard.getBoolean("Follow Target", false);
}

 

@Override

public void teleopInit() {
  //turn on the lights
  //pixycam.setLamp((byte) 1, (byte) 1);
  //Center the servos
  pixycam.setServos(500, 500);
}

@Override
public void teleopPeriodic () {

  
 
  m_drive.arcadeDrive(stick.getY(), stick.getX());
  
  //SmartDashboard.putBoolean( "Camera" , isCamera); //publish if we are connected
   pixycam.getCCC().getBlocks( false , 255 , 255 ); //run getBlocks with arguments to have the camera
    //SmartDashboard.putNumber( "Stuff" , pixycam.getCCC().getBlocks() );
  //acquire target data
    ArrayList<Block> blocks = pixycam.getCCC().getBlockCache(); //assign the data to an ArrayList for convinience
    if (blocks.size() > 0 )
      {
        //Get the blocks
       // String data = blocks.get( 0 ).toString(); // string containing target info
       // double xcoord = blocks.get( 0 ).getX(); // x position of the largest target
        //double ycoord = blocks.get( 0 ).getY(); // y position of the largest target
        
        //Calculate error for pan
        panOffset = pixycam.getFrameWidth()/2 - blocks.get(0).getX();
        tiltOffset = pixycam.getFrameHeight()/2 - blocks.get(0).getY();
        //Send the error to the PID for it to work its magic
        panPID.setError(panOffset);
        tiltPID.setError(tiltOffset);
        panPID.calculateMyPID();
        tiltPID.calculateMyPID();
        //Retrive the desired position (Pan)
        panDesiredPosition = panPID.getDesiredPosition();
        tiltDesiredPosition = tiltPID.getDesiredPosition(); 

        pixycam.setServos(panDesiredPosition, tiltDesiredPosition);
        driveXP = SmartDashboard.getNumber("DriveXP", -.2);

        double sendIt = driveXP;
        

        SmartDashboard.putNumber("desiredPosition",panDesiredPosition);
      //Drive
      

        //double driveXVelocity = MathUtil.clamp(drivePIDX.calculate((panOffset/100), 0), -.8, .8);
        

        

        if (stick.getRawButton(3)){

          convertDriveSpeed.setInput(panDesiredPosition);
          convertDriveSpeed.convert();
          double driveError = convertDriveSpeed.getOutput();
          double steeringAdjust = driveXP*driveError;
          SmartDashboard.putNumber("SteeringAdjust",steeringAdjust);

          if (driveError>0){
            steeringAdjust = driveXP*driveError-minCommand;

          } else if (driveError<0){
            steeringAdjust = driveXP*driveError+minCommand;
          }
          leftSpeed += steeringAdjust;
          MathUtil.clamp(leftSpeed, -0.8, 0.8);
          rightSpeed -= steeringAdjust;
          MathUtil.clamp(rightSpeed,-0.8,0.8);

          m_drive.tankDrive(leftSpeed, rightSpeed);
          SmartDashboard.putNumber("Motor",leftSpeed );
          
        }
        if (stick.getRawButton(4)){

          m_drive.tankDrive(minCommand, minCommand);
        }


      
      //m_drive.arcadeDrive(0, 0);*/


    SmartDashboard.putBoolean( "present" , true ); // show there is a target present
    SmartDashboard.putNumber( "pan offset" ,panOffset);
    //SmartDashboard.putNumber( "Ycoord" , ycoord);
    //SmartDashboard.putString( "Data" , data );
    SmartDashboard.putNumber( "pan" , panDesiredPosition );
      SmartDashboard.putNumber("blocks", blocks.size());


    }
    
    }
}