/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */ 
  
   //This is a Falcon 500. Not sure if FX will work. 
   public WPI_TalonFX leftClimbMotor = new WPI_TalonFX(Constants.leftClimbMotor); 
   public WPI_TalonFX rightClimbMotor = new WPI_TalonFX(Constants.rightClimbMotor); 
   
  //Regular bois go here: 
   public WPI_TalonSRX buddyBarLiftMotorLeft = new WPI_TalonSRX(Constants.buddyBarLiftMotorLeft); 
   public WPI_TalonSRX buddyBarLiftMotorRight = new WPI_TalonSRX(Constants.buddyBarLiftMotorRight); 
   public WPI_TalonSRX frontClampMotorLeft = new WPI_TalonSRX(Constants.frontClampMotorLeft); 
   public WPI_TalonSRX backClampMotorLeft = new WPI_TalonSRX(Constants.backClampMotorLeft); 
   public WPI_TalonSRX frontClampMotorRight = new WPI_TalonSRX(Constants.frontClampMotorRight); 
   public WPI_TalonSRX backClampMotorRight = new WPI_TalonSRX(Constants.backClampMotorRight); 

  //Configurable values for the climb motor: 
  public double reachPosition = 0.0; 
  public double homePosition = 0.0; 

  public Climber() { 

    SmartDashboard.putNumber("Climb Position", reachPosition); 
    SmartDashboard.putNumber("Home Position", homePosition); 

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  } 

  public void reachUp(){  
    leftClimbMotor.set(ControlMode.Position, SmartDashboard.getNumber("Climb Position", reachPosition));
  } 

  public void goHome(){ 
    rightClimbMotor.set(ControlMode.Position, SmartDashboard.getNumber("Home Position", homePosition));  
  }
}
