/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * This class handles the recieving of serial packets from the LIDARs through
 * the serial port.
 * The format of the packet is as follows:
 * "$<int1>,<int2>,<int3>,<int4>\n"
 * Where the '$' indicates the start of the packet, <int1> indicates the first
 * value, <int2> the second value, and so on with the '\n' indicating the end
 * of the packet.
 */

 
public class LidarReader extends Thread {
    SerialPort serialPort;
    boolean keepRunning;

    public LidarReader() {
        serialPort = new SerialPort(Constants.Lidar.BAUD_RATE, Constants.Lidar.PORT, Constants.Lidar.DATA_BITS, Constants.Lidar.PARITY, Constants.Lidar.STOP_BITS);
        keepRunning = true;
    }

    /**
     * Run the Thread loop until killThread() is called.
     */
    @Override
    public void run() {
        // Initialize four LIDAR variables (TODO: Rename properly)
        double[] doubleArray = new double[4];
        serialPort.enableTermination('\n');
        serialPort.setReadBufferSize(64);
        SmartDashboard.putNumberArray(Constants.Lidar.LIDAR_KEY, doubleArray);   
        while(true) {
            // Read String from serial port
            String buffer = serialPort.readString();
            // Ensure that the packet is complete by checking first and last char of String
            if(!buffer.isEmpty() && buffer.charAt(0) == '$' && buffer.charAt(buffer.length() - 1) == '\n') {
                // Split buffer by delimiter ','
                buffer = buffer.substring(1, buffer.length() - 1);
                String[] stringArray = buffer.split(",");
                for(int i = 0; i < stringArray.length; i++)
                    doubleArray[i] = Double.parseDouble(stringArray[i]);
                // Put array into smart dashboard
                SmartDashboard.putNumberArray(Constants.Lidar.LIDAR_KEY, doubleArray);      
            }
        }
    }

    @Override
    public void start() {
        super.start();
        this.keepRunning = true;
    }
    
    public double[] getLidarValues() {
        return SmartDashboard.getNumberArray(Constants.Lidar.LIDAR_KEY, new double[0]);
    }

    public double getLidarValue(int lidar) {
        double values[] = SmartDashboard.getNumberArray(Constants.Lidar.LIDAR_KEY, new double[lidar+1]);
        if(lidar >= values.length){
            return 0;
        }
        return values[lidar];
    }

    public boolean isRunning() {
        return keepRunning;
    }

    public void killThread() {
        this.keepRunning = false;
    }

}