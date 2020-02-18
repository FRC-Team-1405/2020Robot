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
    
    public AddressableLEDBuffer getLedBuffer(){
        return ledBuffer;
    }

    public void displayLEDBuffer(AddressableLEDBuffer ledBuffer){
        this.ledBuffer = ledBuffer;
        led.setData(ledBuffer);
    }


    //colors for climbing
    public void red1(){
        for (int ledIndex = 0 ; ledIndex < ledCount/2 ; ledIndex++){
            setColor(new Color8Bit(255, 0, 0), ledIndex);
        }
    }
    public void blue1(){
        for (int ledIndex = 0 ; ledIndex < ledCount/2 ; ledIndex++){
            setColor(new Color8Bit(0, 0, 255), ledIndex);
        }
    }
    public void green1(){
        for (int ledIndex = 0 ; ledIndex < ledCount/2 ; ledIndex++){
            setColor(new Color8Bit(0, 255, 0), ledIndex);
        }
    }
    public void red2(){
        for (int ledIndex = ledCount/2 ; ledIndex < ledCount ; ledIndex++){
            setColor(new Color8Bit(255, 0, 0), ledIndex);
        }
    }
    public void blue2(){
        for (int ledIndex = ledCount/2 ; ledIndex < ledCount ; ledIndex++){
            setColor(new Color8Bit(0, 0, 255), ledIndex);
        }
    }
    public void green2(){
        for (int ledIndex = ledCount/2 ; ledIndex < ledCount ; ledIndex++){
            setColor(new Color8Bit(0, 255, 0), ledIndex);
        }
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
        ledBuffer.setRGB(i, 0, Math.abs(255-i) % 100, i % 100);
     }
     led.setData(ledBuffer);
     led.start();
   }

}
