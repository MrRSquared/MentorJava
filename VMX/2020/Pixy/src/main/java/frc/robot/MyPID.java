/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * This was modified from the Pixy 2 Arduino code found here. 
 * If we only want one instance of the PID, we can use this inline or as a method. 
 * However, to do Pan and Tilt, we need two instances.  Therefore, we needed to move this to a class.  
 */
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class MyPID {

    private int PID_MAXIMUM_INTEGRAL      =  2000;
    private int PID_MINIMUM_INTEGRAL      = -2000;
    private int m_pgain = 300;
    private int m_igain = 0;
    private int m_dgain = 600;
    private int m_integral;
    private int m_prevError = 0;
    private int m_desiredPosition; //this is the default position (Pixy calls it m_command so as to use it for velocity too)

    public int getDesiredPosition() {
        return this.m_desiredPosition;
    }

    public void setDesiredPosition(int m_desiredPosition) {
        
    }
    private int error;

    public void setError(int error) {
        this.error = error;
    };

    public void setPIDs(int pgain, int igain, int dgain) {
        this.m_pgain = pgain;
        this.m_igain = igain;
        this.m_dgain = dgain;
    }

    int calculateMyPID (){
    //if (m_prevError != 0x80000000){ //Check to see if we need to use PID
        m_integral += error;
        if (m_integral>PID_MAXIMUM_INTEGRAL)
        m_integral = PID_MAXIMUM_INTEGRAL;
        else if (m_integral<PID_MINIMUM_INTEGRAL)
        m_integral = PID_MINIMUM_INTEGRAL;
        //calculate the actual PID.
        int pid = (error*m_pgain + ((m_integral*m_igain)>>4) + (error - m_prevError)*m_dgain)>>10;
  
        m_desiredPosition += pid; // since servo is a position device, we integrate the pid term
                if (m_desiredPosition >1000) //clamp to range max min position.
                  m_desiredPosition = 1000; 
                else if (m_desiredPosition< 0) 
                  m_desiredPosition = 0;
                  m_prevError = error; 
        
      //  }

      SmartDashboard.putNumber( "pidPan" , m_desiredPosition );

        return m_desiredPosition;

    }
    



    void reset()
    {
        m_desiredPosition = 500;
        m_integral = 0;
        m_prevError = 0x80000000;
    } 

}
