/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  // Store what the last hue of the first pixel is
  private int m_rainbowFirstPixelHue;
  // We cannot use a sleep or delay method 
  // because that would leave the robot unresponsive for a spell
  private Timer LEDtimer = new Timer();
  // 
  int count = 0;
  int strandSolidColor = 180;
  int wipeColor = 137;
  double currentTime;


  @Override
  public void robotInit() {
    // PWM port 9 on the Rio, 26 on the VMX-Pi
    // Must be a PWM header, not MXP or DIO
    // On the VMX-Pi it must be the MOSI pin on the VMX
    m_led = new AddressableLED(26);


    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(30);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
    rainbow();
    m_led.setData(m_ledBuffer);
    LEDtimer.start();

  }

  @Override
  public void robotPeriodic() {
    currentTime = LEDtimer.get();
    // Fill the buffer with a rainbow
    //rainbow();
    strandWipe();
    // Set the LEDs
    m_led.setData(m_ledBuffer);
    SmartDashboard.putNumber("Current Time", currentTime);
  }

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 5);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }
  private void strandWipe() {
    // This is a basic wipe algorithm
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Set the strand one color
      m_ledBuffer.setHSV(i, strandSolidColor, 255, 5);
    }
    // Set one pixel a different color.
    m_ledBuffer.setHSV(count, wipeColor, 255, 5);
    // Check the elapsed time.
    if (currentTime  >= .25 ){
    // Move the pixel up the strand
    count +=1;
    // Reset the timer to ensure we move the pixel every quarter of a second
    LEDtimer.reset();
    //Start the process over from the beginning when we hit the end of the strand
    if (count >= m_ledBuffer.getLength()){
      count = 1;
    }
  }
  
  }
  
}
