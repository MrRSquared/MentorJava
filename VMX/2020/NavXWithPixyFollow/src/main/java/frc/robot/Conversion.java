/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * This is a simple conversion class.
 */

public class Conversion{
    //instance variables
    private float input;


    private float inputRangeMax;
    private float inputRangeMin;
    private float inputRange;

    private float output;

    private float outputRangeMin;
    private float outputRangeMax;
    private float outputRange;



    //constructor
    public Conversion(float input, float inputRangeMax, float inputRangeMin, 
     float outputRangeMax, float outputRangeMin){

        this.input = input;
        this.inputRangeMax = inputRangeMax;
        this.inputRangeMin = inputRangeMin;
        this.outputRangeMax = outputRangeMax;
        this.outputRangeMin = outputRangeMin;
    }

    public void convert(){
        this.inputRange = inputRangeMax-inputRangeMin;
        this.outputRange = outputRangeMax-outputRangeMin;
       
        this.output = ((input/inputRange)*outputRange)+outputRangeMin;

    }
    
    public void setInput(float input) {
        this.input = input;
    }

    public float getOutput() {
        return this.output;
    }
}

