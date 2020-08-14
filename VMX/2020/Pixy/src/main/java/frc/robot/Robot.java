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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.pseudoresonance.pixy2api.*;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.I2CLink;
//Import our custom PID class from our robot.
import frc.robot.MyPID;
//import our conversion class

/**
 * PIXY I2C EXAMPLE A few notes 1.) Two things need to be added to the
 * build.gradle file. See this project's build file and the comments therin. 2.) This code is for
 * using I2C. As such, you need to set the Pixy to I2C in PixyMon.
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


  private boolean followToggle;

  Victor m_left = new Victor(0);
  Victor m_right = new Victor(1);
  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);


  //Joystick
  private Joystick stick = new Joystick(0);

@Override
public void robotInit () {

 pixycam = Pixy2.createInstance(new I2CLink()); // Creates a new Pixy2 camera using SPILink
 pixycam.init(); // Initializes the camera and prepares to send/receive data
 panPID = new MyPID();
 panPID.setPIDs(panP, panI, panD);
 tiltPID = new MyPID();
 tiltPID.setPIDs(tiltP, tiltI, tiltD);  
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
       // double xcoord = blocks.get( 0 ).getX(); 
        //double ycoord = blocks.get( 0 ).getY(); 
        
        //Calculate error for pan
        panOffset = pixycam.getFrameWidth()/2 - blocks.get(0).getX();  // x position of the largest target
        tiltOffset = pixycam.getFrameHeight()/2 - blocks.get(0).getY(); // y position of the largest target
        //Send the error to the PID for it to work its magic
        panPID.setError(panOffset);
        tiltPID.setError(tiltOffset);
        panPID.calculateMyPID();
        tiltPID.calculateMyPID();
        //Retrive the desired position (Pan)
        panDesiredPosition = panPID.getDesiredPosition();
        tiltDesiredPosition = tiltPID.getDesiredPosition(); 

        pixycam.setServos(panDesiredPosition, tiltDesiredPosition);
        

        SmartDashboard.putNumber("desiredPosition",panDesiredPosition);
      

        SmartDashboard.putBoolean( "present" , true ); // show there is a target present
        SmartDashboard.putNumber( "pan offset" ,panOffset);
        SmartDashboard.putNumber( "pan" , panDesiredPosition );
        SmartDashboard.putNumber("blocks", blocks.size());


    }
    
    }
}