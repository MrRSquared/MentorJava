/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * This Lidar Class comes from @Glassmaker on FRC Team 103.  Thank you.
 */
import java.nio.ByteBuffer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;



public class Lidar {
	
	private I2C i2c;
	private java.util.Timer updater;
	private ByteBuffer buffer = ByteBuffer.allocateDirect(3);
	private volatile int distance;
	private int measurementCount = 0;
	
	private static final int LIDAR_BUSY_MASK = 0x01;
	private static final int LIDAR_COMMAND_ACQUIRE_WITHOUT_CORRECTION = 0x03;
	private static final int LIDAR_COMMAND_ACQUIRE_WITH_CORRECTION = 0x04;
	private static final int LIDAR_CONFIG_REGISTER = 0x00;
	private static final int LIDAR_STATUS_REGISTER = 0x01;
	private static final int LIDAR_SIG_COUNT = 0x02;
	private static final int LIDAR_ACQ_CONFIG = 0x04;
	private static final int LIDAR_THRESHOLD_BYPASS = 0x1c;
	private static final int LIDAR_DISTANCE_REGISTER = 0x8f;
	
	private static final int UPDATE_PERIOD = 20; // in milliseconds
	private static final int RETRY_COUNT = 50;
	
	public Lidar(Port port, byte address) {
		i2c = new I2C(port, address);
		
		setup();
		
		updater = new java.util.Timer();
		updater.schedule(new TimerTask() {
			@Override
			public void run() {
				distance = getUpdatedDistance();
			}
		}, 0, UPDATE_PERIOD);
		System.out.println("Started");
	}
	
	// Distance in cm
	public int getDistance() {
		return distance;
	}
	
	public void setup() {
		i2c.write(LIDAR_SIG_COUNT, 0x80);
		sleep(1);
		i2c.write(LIDAR_ACQ_CONFIG, 0x08);
		sleep(1);
		i2c.write(LIDAR_THRESHOLD_BYPASS, 0x00);
		sleep(1);
	}
	
	// Update distance variable
	private int getUpdatedDistance() {
		int command = (measurementCount % 100 == 0 ? LIDAR_COMMAND_ACQUIRE_WITH_CORRECTION : LIDAR_COMMAND_ACQUIRE_WITHOUT_CORRECTION);
		i2c.write(LIDAR_CONFIG_REGISTER, command); // Initiate measurement
		/*if (measurementCount++ % 50 == 0) {
			System.out.println("count = " + measurementCount + ", distance = " + distanceValue);
		}*/
		measurementCount++;
		int busyCount = 0;
		do {
			sleep(1);
			int status = readByte(LIDAR_STATUS_REGISTER);
			boolean busy = (status & LIDAR_BUSY_MASK) == LIDAR_BUSY_MASK;
			if (!busy) {
				return readShort(LIDAR_DISTANCE_REGISTER);
			} else {
				busyCount++;
			}
			/*SmartDashboard.putNumber("status", status);
			SmartDashboard.putBoolean("busyFlag", busy);*/
		} while (busyCount < RETRY_COUNT);
		System.out.println("Distance read timed out");
		return distance;
	}
	
	private void sleep(long millis) {
		try { Thread.sleep(millis); } catch (InterruptedException e) { e.printStackTrace(); }
	}
	
	private int readByte(int register) {
		buffer.put(0, (byte) register);
		i2c.writeBulk(buffer, 1);
		i2c.readOnly(buffer, 1);
		return buffer.get(0) & 0xFF;
	}
	
	private int readShort(int register) {
		buffer.put(0, (byte) register);
		i2c.writeBulk(buffer, 1);
		i2c.readOnly(buffer, 2);
		return buffer.getShort(0) & 0xFFFF;
	}
	
}
