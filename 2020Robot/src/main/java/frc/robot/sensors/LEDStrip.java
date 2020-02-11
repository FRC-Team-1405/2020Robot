/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Add your docs here.
 */
public class LEDStrip {
    SPI port; 
    int ledCount; 
    byte[] data; 

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    public LEDStrip(int port, int ledCount){
        led = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(ledCount);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }
    
    public LEDStrip(SPI.Port portID, int ledCount){
        port = new SPI(portID); 
        this.ledCount = ledCount;  
        data = new byte[ledCount * 4 + 8]; 
        data[0] = (byte) 0x00; 
        data[1] = (byte) 0x00; 
        data[2] = (byte) 0x00;  
        data[3] = (byte) 0x00;  
        data[data.length-1] = (byte) 0xFF; 
        data[data.length-2] = (byte) 0xFF; 
        data[data.length-3] = (byte) 0xFF;  
        data[data.length-4] = (byte) 0xFF;  

        Color8Bit finneyGreen = new Color8Bit(0, 64, 0);
        Color8Bit finneyGold = new Color8Bit(230, 185, 5);
        for (int ledIndex = 0 ; ledIndex < ledCount ; ledIndex++){
            if (ledIndex % 3 == 0)
                setColor(finneyGold, ledIndex);
            else
                setColor(finneyGreen, ledIndex);
        }
    }

    //colors for climbing
    public void red1(){

    }
    public void blue1(){

    }
    public void green1(){

    }
    public void red2(){

    }
    public void blue2(){

    }
    public void green2(){

    }

    public void setColor(Color8Bit color, int index){ 
        if(index >= ledCount)
            return; 
        
        int dataIndex = index * 4 + 4;
        data[dataIndex++] = (byte) 0xEF; 
        data[dataIndex++] = (byte) color.blue; 
        data[dataIndex++] = (byte) color.green; 
        data[dataIndex++] = (byte) color.red;  
    }

    public void setColor(Color8Bit[] colors){ 
        if(colors.length >= ledCount)
            return; 
        
        for(int colorIndex = 0; colorIndex < colors.length; colorIndex++){
            int dataIndex = colorIndex * 4 + 4;
            data[dataIndex++] = (byte) 0xFE; 
            data[dataIndex++] = (byte) colors[colorIndex].blue; 
            data[dataIndex++] = (byte) colors[colorIndex].green; 
            data[dataIndex++] = (byte) colors[colorIndex].red;  
        }
    } 

   public void display(){ 
        port.write(data, data.length);
   }

   public void testLEDs(){
    for (var i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, 0, 255-i, i);
     }
     led.setData(ledBuffer);
     led.start();
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
                              (byte) 0xFE, (byte) 0xFF, (byte)0x00, (byte)0x00,
                              (byte) 0xFE, (byte) 0x00, (byte)0xFF, (byte)0x00,
                              (byte) 0xFE, (byte) 0x00, (byte)0x00, (byte)0xFF,
                              (byte) 0xFE, (byte) 0xFF, (byte)0xFF, (byte)0x00,
                              (byte) 0xFE, (byte) 0xFF, (byte)0x00, (byte)0xFF,
                              (byte) 0xFE, (byte) 0x00, (byte)0xFF, (byte)0xFF,
                              (byte) 0xFE, (byte) 0xFF, (byte)0xFF, (byte)0xFF,
                              (byte) 0xFE, (byte) 0xFF, (byte)0x00, (byte)0x00,
                              (byte) 0xFE, (byte) 0x00, (byte)0xFF, (byte)0x00,
                              (byte) 0xFE, (byte) 0x00, (byte)0x00, (byte)0xFF,
                              (byte) 0xFE, (byte) 0xFF, (byte)0xFF, (byte)0x00,
                              (byte) 0xFE, (byte) 0xFF, (byte)0x00, (byte)0xFF,
                              (byte) 0xFE, (byte) 0x00, (byte)0xFF, (byte)0xFF,
                              (byte) 0xFE, (byte) 0xFF, (byte)0xFF, (byte)0xFF,
                              (byte) 0xFE, (byte) 0xFF, (byte)0x00, (byte)0x00,
                              (byte) 0xFE, (byte) 0x00, (byte)0xFF, (byte)0x00,
                              (byte) 0xFE, (byte) 0x00, (byte)0x00, (byte)0xFF,
                              (byte) 0xFE, (byte) 0xFF, (byte)0xFF, (byte)0x00,
                              (byte) 0xFE, (byte) 0xFF, (byte)0x00, (byte)0xFF,
                              (byte) 0xFE, (byte) 0x00, (byte)0xFF, (byte)0xFF,
                              (byte) 0xFE, (byte) 0xFF, (byte)0xFF, (byte)0xFF,
                              (byte) 0xFE, (byte) 0xFF, (byte)0x00, (byte)0x00,
                              (byte) 0xFF, (byte) 0xFF, (byte)0xFF, (byte)0xFF
                             };
    port.write(data, data.length);
} 

}
