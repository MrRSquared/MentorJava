/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * This is a sample to pass instance variables.
 */

public class getSet {
    private int m_input;
    private int m_output;

    public int getM_input() {
        return this.m_input;
    }

    public void setM_input(int m_input) {
        this.m_input = m_input;
    }

    public int getM_output() {
        return this.m_output;
    }

    public void setM_output(int m_output) {
        this.m_output = m_output;
    }

    private int k_operator = 5;

    void operation(){
        this.m_output = k_operator*m_input;

    }
}
