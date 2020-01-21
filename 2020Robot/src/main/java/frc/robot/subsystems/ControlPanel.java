/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.ColorSensor;
import frc.robot.sensors.FMSData;

public class ControlPanel extends SubsystemBase {
  /**
   * Creates a new ControlPanel.
   */

  WPI_TalonSRX controlMotor = new WPI_TalonSRX(Constants.controlPanel);
  //CANSparkMax controlMotor = new CANSparkMax(Constants.controlPanel, MotorType.kBrushless);

  private ColorSensor sensor = new ColorSensor();
  public ControlPanel() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean checkColor(){
    return FMSData.getColor() == sensor.readColor();
  }

  public void rotationControl(double distance){
    controlMotor.set(ControlMode.Position, distance);
  }

  public void stop() {
    controlMotor.set(ControlMode.PercentOutput, 0);
  }

  private static final int MIN_POSITION_ERROR = 1000;
  public boolean isRotationComplete(){
    return controlMotor.getClosedLoopError() < MIN_POSITION_ERROR;
  }
 
  public void positionControl(){
    controlMotor.set(ControlMode.PercentOutput, Constants.ControlPanelConstants.SPEED);
  }
}
