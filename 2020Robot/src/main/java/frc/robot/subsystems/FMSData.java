/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.ColorSensor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;

public class FMSData extends SubsystemBase {
  /**
   * Creates a new FMSData.
   */
  DriverStation driverStation = DriverStation.getInstance();
  public FMSData() {

  }

  public Color getPositionControlColor(){
    String data = driverStation.getGameSpecificMessage();
    if(data.length() > 0){
      switch (data.charAt(0)){
        case 'B': return ColorSensor.Target.BLUE;
        case 'G': return ColorSensor.Target.GREEN;
        case 'R': return ColorSensor.Target.RED;
        case 'Y': return ColorSensor.Target.YELLOW;
        default: return null;
  
      }
    }
    return null;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
