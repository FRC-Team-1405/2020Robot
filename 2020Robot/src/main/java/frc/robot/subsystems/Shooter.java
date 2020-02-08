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
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.LidarLitePWM;

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

  //public WPI_TalonSRX left = new WPI_TalonSRX(Constants.shooterLeft); 
  //public WPI_TalonSRX right = new WPI_TalonSRX(Constants.shooterRight); 
   public CANSparkMax left = new CANSparkMax(20, MotorType.kBrushless); 
   public CANSparkMax right = new CANSparkMax(21, MotorType.kBrushless); 

  public CANEncoder leftEncoder = new CANEncoder(left); 
  public CANEncoder rightEncoder = new CANEncoder(right); 

  CANPIDController leftPIDController = new CANPIDController(left);  
  CANPIDController rightPIDController = new CANPIDController(right);
  
  private final LidarLitePWM lidarLitePWM = new LidarLitePWM(new DigitalInput(9));

  public Shooter() { 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run 
    //SmartDashboard.putNumber("Left Error", left.getClosedLoopError()); 
    //SmartDashboard.putNumber("Right Error", right.getClosedLoopError()); 
    //For competitons comment out shuffleboard distance and uncomment variable
    SmartDashboard.putNumber("Lidar_Distance", lidarLitePWM.getDistance());
    //lidarLitePWM.getDistance();
  }

  public void launch(double leftDistance, double rightDistance){
    leftPIDController.setReference(leftDistance, ControlType.kVelocity);
    rightPIDController.setReference(rightDistance, ControlType.kVelocity); 
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
    double power = (((high.power - low.power) / (high.distance - low.distance)) * (distance - low.distance) + low.power);
    return power;
  }

  Setting settings[] = new Setting[] {
                                        new Setting(2000, 200),
                                        new Setting(3000, 300),
                                        new Setting(4000, 400), 
                                        new Setting(5000, 500),
                                        new Setting(6000, 600) };

  public void fire(double distance){
    int lowIndex = 0;
    int highIndex = 0;
    double power = 0.0;

    if(distance < settings[0].distance){
      power = settings[0].power;
    }
    else if(distance > settings[settings.length-1].distance){
      power = settings[settings.length-1].power;
    }
    else{
      for(int i = 1; i < settings.length-2; i++){
        if(distance < settings[i].distance){
          lowIndex = i-1;
          highIndex = i;
          power = CalculatePower(distance, settings[lowIndex], settings[highIndex]);
          break;
        }
      }
    }
    left.set(power);
    right.set(power);
    SmartDashboard.putNumber("Shooter_Power", power);
    SmartDashboard.putNumber("Shooter_Distance", distance);
  }

  public void fire(){
    fire(lidarLitePWM.getDistance());
  }
}
