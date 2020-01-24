/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import com.ctre.phoenix.CANifier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LIDARCanifier extends SubsystemBase {
  /**
   * Creates a new LIDARCanifier.
   */
  CANifier canifier;
  public LIDARCanifier(int kCanifierID) { // 5190's code told me to always pass in 16 for this value. I don't know if that's true or not, so take that info with caution.
    canifier = new CANifier(kCanifierID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
