/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Carousel extends SubsystemBase {

  WPI_TalonSRX spinDexer = new WPI_TalonSRX(Constants.spinDexerId);

  //This needs to be between 1 and 0
  private static final double SPEED = 0.1;

  public void clockwise() {
    spinDexer.set(ControlMode.PercentOutput, SPEED);
  }

  public void counterclockwise() {
    spinDexer.set(ControlMode.PercentOutput, -SPEED);
  }

  public void stop() {
    spinDexer.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
