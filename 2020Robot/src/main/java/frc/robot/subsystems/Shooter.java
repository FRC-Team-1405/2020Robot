/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.sensors.Limelight;

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
  public WPI_TalonSRX indexer = new WPI_TalonSRX(Constants.indexerid);
  public WPI_TalonSRX trigger = new WPI_TalonSRX(Constants.triggerid);
  public Limelight limelight = new Limelight();
  private Turret turret;

  // public CANEncoder leftEncoder = new CANEncoder(left); 
  // public CANEncoder rightEncoder = new CANEncoder(right); 

  // CANPIDController leftPIDController = new CANPIDController(left);  
  // CANPIDController rightPIDController = new CANPIDController(right);
  
  public double triggerSpeed = 0.6; 
  public boolean tracking = false;

  public Shooter(Turret turret) { 
    SmartDashboard.putNumber("Trigger Speed", triggerSpeed); 
    this.turret = turret;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run 

    if(!Robot.fmsAttached){
      SmartDashboard.putNumber("Left Error", left.getClosedLoopError()); 
      SmartDashboard.putNumber("Right Error", right.getClosedLoopError()); 
    }else{
      //SmartDashboard.putNumber("Lidar_Distance", lidarLitePWM.getDistance());
    }
  }

  public void launch(double leftDistance, double rightDistance){
    // leftPIDController.setReference(leftDistance, ControlType.kVelocity);
    // rightPIDController.setReference(rightDistance, ControlType.kVelocity); 
    left.set(ControlMode.Velocity, leftDistance); 
    right.set(ControlMode.Velocity, rightDistance); 
    if(!Robot.fmsAttached){
      SmartDashboard.putNumber("Left Distance", leftDistance); 
      SmartDashboard.putNumber("Right Distance", rightDistance); 
    }
  }

  public void stopFlywheels(){
    tracking = false;
    limelight.setPipeline((byte) 0);
    limelight.setLED((byte) 1);
    left.set(ControlMode.PercentOutput, 0.0); 
    right.set(ControlMode.PercentOutput, 0.0); 
    trigger.set(ControlMode.PercentOutput, 0);
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
                                        new Setting(10000, 1.82),
                                        new Setting(10500, 0.97),
                                        new Setting(11000, 0.625), };

  public void prepFlywheels(){
    limelight.setPipeline((byte) 7);
    limelight.setLED((byte) 3);
    tracking = true;
    turret.turnTurret();
    int lowIndex = 0;
    int highIndex = 0;
    double power = 0.0;
    double distance = limelight.getTA();

    if(distance <= settings[0].distance){
      power = settings[0].power;
    }
    else if(distance >= settings[settings.length-1].distance){
      power = settings[settings.length-1].power;
    }
    else{
      for(int i = 1; i < settings.length; i++){
        if(distance <= settings[i].distance){
          lowIndex = i-1;
          highIndex = i;
          power = CalculatePower(distance, settings[lowIndex], settings[highIndex]);
          break;
        }
      }
    }
    left.set(ControlMode.Velocity, -power - RobotContainer.increase);
    right.set(ControlMode.Velocity, power + RobotContainer.increase);
    trigger.set(ControlMode.PercentOutput, -0.6);
    //trigger.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Trigger Speed", triggerSpeed));
  }

  public void prepFlywheels(DoubleSupplier leftV, DoubleSupplier rightV){
    limelight.setPipeline((byte) 7);
    limelight.setLED((byte) 3);
    tracking = true;
    turret.turnTurret();
    prepFlywheels(leftV.getAsDouble() - RobotContainer.increase, rightV.getAsDouble() + RobotContainer.increase);
  } 

  public void prepFlywheels(double leftV, double rightV){
    tracking = true;
    left.set(ControlMode.Velocity, -leftV - RobotContainer.increase);
    right.set(ControlMode.Velocity, rightV + RobotContainer.increase);
    trigger.set(ControlMode.PercentOutput, -0.6);
  }

  public void fire(){
    indexer.set(ControlMode.PercentOutput, 0.6);

  }

  public void stopIndexer(){
    indexer.set(ControlMode.PercentOutput, 0);
    //trigger.set(0);
  }

  public void outtake(){
    indexer.set(ControlMode.PercentOutput, -0.5);
    //trigger.set(ControlMode.PercentOutput, 0.3);
  }

  public boolean flywheelReady() {
    return (Math.abs(left.getClosedLoopError()) <= Constants.maxFlywheelError && Math.abs(right.getClosedLoopError()) <= Constants.maxFlywheelError);
  }
}
