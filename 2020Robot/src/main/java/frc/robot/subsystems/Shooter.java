/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.ControlType;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */ 
    private class Setting {
      public Setting(double pow, double dis) {
        power = pow;
        distance = dis;
      }
      public double power;
      public double distance;
    } ; 

  public WPI_TalonSRX left = new WPI_TalonSRX(Constants.shooterLeft); 
  public WPI_TalonSRX right = new WPI_TalonSRX(Constants.shooterRight); 
  // public CANSparkMax left = new CANSparkMax(20, MotorType.kBrushless); 
  // public CANSparkMax right = new CANSparkMax(21, MotorType.kBrushless); 

  // public CANEncoder leftEncoder = new CANEncoder(left); 
  // public CANEncoder rightEncoder = new CANEncoder(right); 

  // CANPIDController leftPIDController = new CANPIDController(left);  
  // CANPIDController rightPIDController = new CANPIDController(right); 

  public Shooter() { 
    SmartDashboard.putBoolean("Shooter/isReady", false);
    SmartDashboard.putString("Shooter/launch", ""); 



    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run 
    //SmartDashboard.putNumber("Left Error", left.getClosedLoopError()); 
    //SmartDashboard.putNumber("Right Error", right.getClosedLoopError()); 
  }

  public void launch(double leftDistance, double rightDistance){
    SmartDashboard.putString("Shooter/launch", "FIRE");
    SmartDashboard.putBoolean("Shooter/isReady", false); 

    left.set(ControlMode.Velocity, leftDistance); 
    right.set(ControlMode.Velocity, rightDistance);  
    // leftPIDController.setReference(leftDistance, ControlType.kVelocity);
    // rightPIDController.setReference(rightDistance, ControlType.kVelocity); 
  }

  public void stop(){
    left.set(0.0); 
    right.set(0.0);  
  }

  public boolean isReady(){
    boolean ready = SmartDashboard.getBoolean("Shooter/isReady", false);    
    if (ready){
      SmartDashboard.putString("Shooter/launch", "");
    }
    return ready;
  }

  private double CalculatePower(double distance, Setting low, Setting high){ 
    //From slope point formula (y-y_1) = m(x-x_1) -> y = m(x-x_1) + y_1
    double power = ((high.power - low.power) / (high.distance - low.distance)) * ((distance - low.distance) + low.power);
    return power;
  }

  Setting settings[] = new Setting[] {
                                        new Setting(19200, 10),
                                        new Setting(17200, 20),
                                        new Setting(15100, 30), 
                                        new Setting(13100, 40),
                                        new Setting(11100, 50) };

  public void fire(double distance){
    SmartDashboard.putString("Shooter/fire", "FIRE");
    SmartDashboard.putBoolean("Shooter/isReady", false);
    int lowIndex = 0;
    int highIndex = 0;

    if(distance < settings[0].distance){
      lowIndex = 0;
      highIndex = 0;
    }
    else if(distance > settings[settings.length-1].distance){
      lowIndex = settings.length-1;
      highIndex = settings.length-1;
    }
    else{
      for(int i = 1; i < settings.length-2; i++){
        if(distance < settings[i].distance){
          lowIndex = i-1;
          highIndex = i;
          break;
        }
      }
    }
    double power = CalculatePower(distance, settings[lowIndex], settings[highIndex]);
    left.set(power);
    right.set(power);
    
  }
}
