/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */ 
  public WPI_TalonSRX left = new WPI_TalonSRX(Constants.shooterLeft); 
  public WPI_TalonSRX right = new WPI_TalonSRX(Constants.shooterRight); 
  
  public Shooter() {
    SmartDashboard.putBoolean("Shooter/isReady", false);
    SmartDashboard.putString("Shooter/launch", ""); 

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void launch(double distance){
    SmartDashboard.putString("Shooter/launch", "FIRE");
    SmartDashboard.putBoolean("Shooter/isReady", false); 

    left.set(ControlMode.Velocity, distance); 
    right.set(ControlMode.Velocity, -distance);  
  }

  public boolean isReady(){
    boolean ready = SmartDashboard.getBoolean("Shooter/isReady", false);
    if (ready){
      SmartDashboard.putString("Shooter/launch", "");
    }
    return ready;
  }
}
