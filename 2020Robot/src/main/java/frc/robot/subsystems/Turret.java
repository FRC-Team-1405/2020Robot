/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.sensors.LidarLitePWM;
import frc.robot.sensors.Limelight;

public class Turret extends SubsystemBase {
  // initialization variables
  public WPI_TalonSRX turret = new WPI_TalonSRX(Constants.turretid);
  public Servo leftActuator = new Servo(Constants.leftActuatorId);
  public Servo rightActuator = new Servo(Constants.rightActuatorId);
  public Limelight limelight = new Limelight();
/*Other variables*/
  private int withinThresholdLoops = 0;
  private int loopsToSettle = 10;
// other other variables
  private int targetPosition = Constants.ShooterConstants.unitsMin;
// other other other variables
  private int errorThreshold = 25;
//other other other other variables
public final LidarLitePWM lidarLitePWM = new LidarLitePWM(new DigitalInput(9)); 

  public Turret() {
    turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    int pos = turret.getSensorCollection().getPulseWidthPosition(); 
    turret.getSensorCollection().setQuadraturePosition(pos-3623, 0); 
    
    turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Position", turret.getSelectedSensorPosition());
   
    
    
    if (turret.getActiveTrajectoryPosition() == targetPosition && Math.abs(turret.getClosedLoopError()) < errorThreshold){
      withinThresholdLoops++;
    }else{
      withinThresholdLoops = 0;
    } 

    SmartDashboard.putNumber("Lidar_Distance", lidarLitePWM.getDistance());

    if(!Robot.fmsAttached){
      SmartDashboard.putNumber("Limelight/TXPos", limelight.getTXPos());
      SmartDashboard.putNumber("Limelight/TYPos", limelight.getTYPos());
      SmartDashboard.putNumber("Lidar_Distance", lidarLitePWM.getDistance());
    }else{
      //SmartDashboard.putNumber("Lidar_Distance", lidarLitePWM.getDistance());
    }
  }

  public boolean setLow = false;

  public void toggleElevation(){
    setLow = !setLow;
    setElevation();
  }

  public void setElevationManual(double elevation){
    leftActuator.set(elevation);
    rightActuator.set(elevation);
  }

  public void setElevation(){
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
    withinThresholdLoops = 0;
    Pose2d pose = driveBase.getPose();
    double currentX = pose.getTranslation().getX();
    double currentY = pose.getTranslation().getY();
    double heading = pose.getRotation().getDegrees();
    double angle = 90.0 - Math.atan((-currentY)/(Constants.goalX - currentX)) - heading;
    this.turnTurret((int) angle);
  }

  public boolean turretTurnIsComplete(){
    return (withinThresholdLoops > loopsToSettle);
    //return turret.getClosedLoopError() < Constants.ShooterConstants.turretError;
  }
  public void turnTurret(){
    withinThresholdLoops = 0;
    if(limelight.getPipeline() == 7){
      turnTurret(-(int) limelight.getTX());
    }
  };

  public void goHome(){
    turret.set(ControlMode.MotionMagic, Constants.ShooterConstants.turretCenter);
  }

  public void resetEncoder(){
    turret.setSelectedSensorPosition(0);
  }

  public void turnTurret(double angle){
    int currentPos = turret.getSelectedSensorPosition();
    angle = (int) (angle * Constants.ShooterConstants.unitsPerAngle) * 
    1.15;
    if(currentPos+angle < Constants.ShooterConstants.unitsMin){
      targetPosition = Constants.ShooterConstants.unitsMin;
      turret.set(ControlMode.MotionMagic, Constants.ShooterConstants.unitsMin);
    } else if(currentPos+angle > Constants.ShooterConstants.unitsMax){
      targetPosition = Constants.ShooterConstants.unitsMax;
      turret.set(ControlMode.MotionMagic, Constants.ShooterConstants.unitsMax);
    } else{
      turret.set(ControlMode.MotionMagic, currentPos+angle);
      targetPosition = (int) (currentPos+angle);
    }
  }

  public boolean turretReady(){
    return limelight.hasTarget() && (Math.abs(limelight.getTX()) <= Constants.ShooterConstants.limelightError);
  }

  public void stopTurret(){
    turret.set(ControlMode.Position, turret.getSelectedSensorPosition());
  }
}
