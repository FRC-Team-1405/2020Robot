/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import com.ctre.phoenix.CANifier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.CircularBuffer;
import frc.robot.Constants;
import frc.robot.lib.MeanFilter;

public class LIDARCanifier extends SubsystemBase {
  /**
   * Creates a new LIDARCanifier.
   */
  CANifier canifier;
  private double[] tempPWMData;
  private CircularBuffer rollingLidarAverage;
  private double lidarRawAveraged;
  public LIDARCanifier(int kCanifierID) { // 5190's code told me to always pass in 16 for this value. I don't know if that's true or not, so take that info with caution.
    canifier = new CANifier(kCanifierID);
    rollingLidarAverage = new CircularBuffer(Constants.lidarBufferSize);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    canifier.getPWMInput(CANifier.PWMChannel.PWMChannel0, tempPWMData);
    rollingLidarAverage.addFirst(tempPWMData[0]);
    lidarRawAveraged = new MeanFilter(Constants.lidarBufferSize).filter(tempPWMData[0]);
  }
}
