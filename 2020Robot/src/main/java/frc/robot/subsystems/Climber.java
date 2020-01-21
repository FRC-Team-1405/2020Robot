/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */ 
  
   //This is a Falcon 500. Not sure if FX will work. 
   public WPI_TalonFX climbMotor = new WPI_TalonFX(Constants.climbMotor); 
   
  //Regular bois go here: 
   public WPI_TalonSRX buddyBarLiftMotorLeft = new WPI_TalonSRX(Constants.buddyBarLiftMotorLeft); 
   public WPI_TalonSRX buddyBarLiftMotorRight = new WPI_TalonSRX(Constants.buddyBarLiftMotorRight); 
   public WPI_TalonSRX frontClampMotorLeft = new WPI_TalonSRX(Constants.frontClampMotorLeft); 
   public WPI_TalonSRX backClampMotorLeft = new WPI_TalonSRX(Constants.backClampMotorLeft); 
   public WPI_TalonSRX frontClampMotorRight = new WPI_TalonSRX(Constants.frontClampMotorRight); 
   public WPI_TalonSRX backClampMotorRight = new WPI_TalonSRX(Constants.backClampMotorRight); 

  public Climber() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
