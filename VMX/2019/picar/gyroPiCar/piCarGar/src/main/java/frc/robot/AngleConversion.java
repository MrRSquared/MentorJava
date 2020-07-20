/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * This is a little class to convert the joystick angle to the servo on the piCar Steering Yoke.
 */
public class AngleConversion {
    private double inputNumber;
    private double outputNumber;
    private int oldMax;
    private int oldMin;
    private int newMax;
    private int newMin;

    public void setOldRange(int oldLow, int oldHigh){
        this.oldMin = oldLow;
        this.oldMax = oldHigh;
    }
    public void setNewRange(int newLow, int newHigh){
        this.newMin = newLow;
        this.newMax = newHigh;
    }
    public double getOutputNumber(){
        return outputNumber;
    }

    public void setInputNumber(double num){
        this.inputNumber = num;
    }
    public void convert(){
        int oldRange = (oldMax - oldMin);
        int newRange = (newMax - newMin);
        outputNumber = (((inputNumber - oldMin) * newRange) / oldRange) + newMin;
    // newNumber  = someNumber *2;

    }
}
