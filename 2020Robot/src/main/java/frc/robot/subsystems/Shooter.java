/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.revrobotics.CANEncoder;
// import com.revrobotics.CANPIDController;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.ControlType;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.MathTools;
import frc.robot.sensors.LidarLitePWM;
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
  public WPI_TalonSRX turret = new WPI_TalonSRX(Constants.turretid);
  public WPI_TalonSRX indexer = new WPI_TalonSRX(Constants.indexerid);
  public WPI_TalonSRX trigger = new WPI_TalonSRX(Constants.triggerid);
  public Servo leftActuator = new Servo(Constants.leftActuatorId);
  public Servo rightActuator = new Servo(Constants.rightActuatorId);
  
  public Limelight limelight = new Limelight();
  //  public CANSparkMax left = new CANSparkMax(20, MotorType.kBrushless); 
  //  public CANSparkMax right = new CANSparkMax(21, MotorType.kBrushless); 

  // public CANEncoder leftEncoder = new CANEncoder(left); 
  // public CANEncoder rightEncoder = new CANEncoder(right); 

  // CANPIDController leftPIDController = new CANPIDController(left);  
  // CANPIDController rightPIDController = new CANPIDController(right);
  
  public final LidarLitePWM lidarLitePWM = new LidarLitePWM(new DigitalInput(9)); 
  public double triggerSpeed = 0.5; 

  public Shooter() { 
    SmartDashboard.putNumber("Trigger Speed", triggerSpeed); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run 
    SmartDashboard.putNumber("Left Error", left.getClosedLoopError()); 
    SmartDashboard.putNumber("Right Error", right.getClosedLoopError()); 
    //For competitons comment out shuffleboard distance and uncomment variable
    SmartDashboard.putNumber("Lidar_Distance", lidarLitePWM.getDistance());
    //lidarLitePWM.getDistance();
  }

  public void launch(double leftDistance, double rightDistance){
    // leftPIDController.setReference(leftDistance, ControlType.kVelocity);
    // rightPIDController.setReference(rightDistance, ControlType.kVelocity); 
    left.set(ControlMode.Velocity, leftDistance); 
    right.set(ControlMode.Velocity, rightDistance); 
    SmartDashboard.putNumber("Left Distance", leftDistance); 
    SmartDashboard.putNumber("Right Distance", rightDistance); 
  }

  public void stopFlywheels(){
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
                                        new Setting(16000, 1),
                                        new Setting(18000, 2),
                                        new Setting(20000, 3), 
                                        new Setting(22000, 4) };

  public void prepFlywheels(){
    prepFlywheels(lidarLitePWM.getDistance());
    //trigger.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Trigger Speed", triggerSpeed));  
    trigger.set(ControlMode.PercentOutput, -0.5);
  }


  public void prepFlywheels(double distance){
    // int lowIndex = 0;
    // int highIndex = 0;
    // double power = 0.0;

    // if(distance < settings[0].distance){
    //   power = settings[0].power;
    // }
    // else if(distance > settings[settings.length-1].distance){
    //   power = settings[settings.length-1].power;
    // }
    // else{
    //   for(int i = 1; i < settings.length-2; i++){
    //     if(distance < settings[i].distance){
    //       lowIndex = i-1;
    //       highIndex = i;
    //       power = CalculatePower(distance, settings[lowIndex], settings[highIndex]);
    //       break;
    //     }
    //   }
    // }
    // left.set(-power);
    // right.set(power);
    left.set(ControlMode.Velocity, -distance);
    right.set(ControlMode.Velocity, distance);
    trigger.set(ControlMode.PercentOutput, -0.5);
    // SmartDashboard.putNumber("Shooter_Power", power);
    // SmartDashboard.putNumber("Shooter_Distance", distance);
  }

  public void fire(){
    indexer.set(ControlMode.PercentOutput, 0.6);

  }

  public void stopIndexer(){
    indexer.set(ControlMode.PercentOutput, 0);
  }

  public void outtake(){
    indexer.set(ControlMode.PercentOutput, -0.5);
    trigger.set(ControlMode.PercentOutput, 0.3);
  }

  public boolean flywheelReady() {
    return left.getClosedLoopError() <= Constants.maxFlywheelError && right.getClosedLoopError() <= Constants.maxFlywheelError ;
  }

  public boolean setLow = false;

  public void toggleElevation(){
    setLow = !setLow;
  }

  public void setElevationManual(double elevation){
    leftActuator.set(elevation);
    rightActuator.set(elevation);
  }

  public void setElevationMin(){
    if(setLow){
      leftActuator.set(Constants.ShooterConstants.elevationMin);
      rightActuator.set(Constants.ShooterConstants.elevationMin);
    }else{
      leftActuator.set(Constants.ShooterConstants.elevationMax); 
      rightActuator.set(Constants.ShooterConstants.elevationMax);
    }

  }

  public boolean hasTarget(){
    return limelight.hasTarget();
  }

  public void turnToGoal(ArcadeDrive driveBase){
    Pose2d pose = driveBase.getPose();
    double currentX = pose.getTranslation().getX();
    double currentY = pose.getTranslation().getY();
    double heading = pose.getRotation().getDegrees();
    double angle = 90.0 - Math.atan((-currentY)/(Constants.goalX - currentX)) - heading;
    this.turnTurret((int) angle);
  }

  public boolean turretTurnIsComplete(){
    return turret.getClosedLoopError() < Constants.ShooterConstants.turretError;
  }
  public void turnTurret(){
    turnTurret((int) limelight.getTX());
  };

  public void turnTurret(int angle){
    int currentPos = turret.getSelectedSensorPosition();
    angle = MathTools.map(angle, Constants.ShooterConstants.angleMin, Constants.ShooterConstants.angleMax, Constants.ShooterConstants.unitsMin, Constants.ShooterConstants.unitsMax);
    if(currentPos+angle < Constants.ShooterConstants.unitsMin){
      turret.set(ControlMode.Position, Constants.ShooterConstants.unitsMin);
    } else if(currentPos+angle > Constants.ShooterConstants.unitsMax){
      turret.set(ControlMode.Position, Constants.ShooterConstants.unitsMax);
    } else{
      turret.set(ControlMode.Position, currentPos+angle);
    }
  }

  public boolean turretReady(){
    return Math.abs(limelight.getTX()) <= Constants.ShooterConstants.limelightError;
  }
}
