/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;

import edu.wpi.first.wpilibj.DigitalInput;

public class RobotConfigBoard {

    public RobotConfigBoard(int startInput) {
        /// for loop
        // create new input using loop index
       DigitalInput inputs[] = new DigitalInput[8];
        for (int i=0; i<8; i++){
        inputs [i] = new DigitalInput(startInput + i);
        }

        boolean outputs[] = new boolean[8];
            
        for (int o=0; o<8; o++){
            outputs[o] = inputs[o].get();
            System.out.println(inputs[o].get());
        };
    }    
}
