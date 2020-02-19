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

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */

  WPI_TalonSRX intakeTalon = new WPI_TalonSRX(Constants.intakeTalon);
  WPI_TalonSRX intakeDeploy = new WPI_TalonSRX(Constants.intakeDeploy);

  public Intake() {
    intakeTalon.set(ControlMode.PercentOutput, 0);
    intakeDeploy.set(ControlMode.Position, Constants.IntakeConstants.RETRACT_POSITION);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void deploy(){
    intakeDeploy.set(ControlMode.Position, Constants.IntakeConstants.DEPLOY_POSITION);
  } 

  public void deployRendezvous(){ 
    intakeDeploy.set(ControlMode.Position, Constants.IntakeConstants.RENDEZVOUS_POSITION); 
  }

  public void retract(){
    intakeDeploy.set(ControlMode.Position, Constants.IntakeConstants.RETRACT_POSITION);
  } 

  public void enable(){
    intakeTalon.set(ControlMode.PercentOutput, Constants.IntakeConstants.SPEED);
  }

  public void disable(){
    intakeTalon.set(ControlMode.PercentOutput, 0);
  }
}
