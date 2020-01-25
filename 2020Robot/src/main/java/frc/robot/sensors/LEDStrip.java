/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.SPI;

/**
 * Add your docs here.
 */
public class LEDStrip {
    SPI port;
    public LEDStrip(SPI.Port portID){
        port = new SPI(portID);
    }

    public void testOn(){
        byte[] data = new byte[] {(byte) 0x00, (byte) 0x00, (byte)0x00, (byte)0x00,
                                  (byte) 0xFE, (byte) 0xFF, (byte)0x00, (byte)0x00,
                                  (byte) 0xFE, (byte) 0x00, (byte)0xFF, (byte)0x00,
                                  (byte) 0xFE, (byte) 0x00, (byte)0x00, (byte)0xFF,
                                  (byte) 0xFE, (byte) 0xFF, (byte)0xFF, (byte)0x00,
                                  (byte) 0xFE, (byte) 0xFF, (byte)0x00, (byte)0xFF,
                                  (byte) 0xFE, (byte) 0x00, (byte)0xFF, (byte)0xFF,
                                  (byte) 0xFE, (byte) 0xFF, (byte)0xFF, (byte)0xFF,
                                  (byte) 0xFF, (byte) 0xFF, (byte)0xFF, (byte)0xFF
                                 };
        port.write(data, data.length);
    }
}
