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

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArcadeDrive extends SubsystemBase {
  /**
   * Creates a new ArcadeDrive.
   */
  // WPI_TalonSRX driveLeft = new WPI_TalonSRX(Constants.driveLeft);
  // WPI_TalonSRX driveRight = new WPI_TalonSRX(Constants.driveRight);
  // WPI_TalonSRX driveLeftSlave1 = new WPI_TalonSRX(Constants.driveLeftSlave1);
  // WPI_TalonSRX driveRightSlave1 = new WPI_TalonSRX(Constants.driveRightSlave1);
  // WPI_TalonSRX driveLeftSlave2 = new WPI_TalonSRX(Constants.driveLeftSlave2);
  // WPI_TalonSRX driveRightSlave2 = new WPI_TalonSRX(Constants.driveRightSlave2);
  // DifferentialDrive driveBase = new DifferentialDrive(driveLeft, driveRight);
  CANSparkMax frontLeft = new CANSparkMax(Constants.driveLeft, MotorType.kBrushless);
  CANSparkMax frontRight = new CANSparkMax(Constants.driveRight, MotorType.kBrushless);
  CANSparkMax backLeft = new CANSparkMax(Constants.driveLeftSlave1, MotorType.kBrushless);
  CANSparkMax backRight = new CANSparkMax(Constants.driveRightSlave1, MotorType.kBrushless);
  DifferentialDrive driveBase = new DifferentialDrive(frontLeft, frontRight);
  // WPI_TalonSRX driveLeft = new WPI_TalonSRX(Constants.driveLeft);
  // WPI_TalonSRX driveRight = new WPI_TalonSRX(Constants.driveRight);
  // WPI_TalonSRX driveLeftSlave1 = new WPI_TalonSRX(Constants.driveLeftSlave1);
  // WPI_TalonSRX driveRightSlave1 = new WPI_TalonSRX(Constants.driveRightSlave1);
  // WPI_TalonSRX driveLeftSlave2 = new WPI_TalonSRX(Constants.driveLeftSlave2);
  // WPI_TalonSRX driveRightSlave2 = new WPI_TalonSRX(Constants.driveRightSlave2);
  // DifferentialDrive driveBase = new DifferentialDrive(driveLeft, driveRight);
  private AHRS gyro = new AHRS(I2C.Port.kMXP);
  boolean driveForward = true;
  
  public ArcadeDrive() {
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);
    // driveLeft.configNeutralDeadband(0.04, 10);
    // driveRight.configNeutralDeadband(0.04, 10);

    // driveLeft.set(ControlMode.PercentOutput, 0);
    // driveRight.set(ControlMode.PercentOutput, 0);
    // driveLeftSlave1.set(ControlMode.Follower, Constants.driveLeft);
    // driveRightSlave1.set(ControlMode.Follower, Constants.driveRight);
    // driveLeftSlave2.set(ControlMode.Follower, Constants.driveLeft);
    // driveRightSlave2.set(ControlMode.Follower, Constants.driveRight);

    driveBase.setDeadband(0.0);
  }

  public void toggleDriveDirection(){
    driveForward = !driveForward;
  }

  public void driveRobot(double xSpeed, double zRotation){
    if(driveForward){
      driveBase.arcadeDrive(xSpeed, zRotation);
    }else{
      driveBase.arcadeDrive(-xSpeed, zRotation);
    }
  }

  public double getVelocity(){
   return Math.sqrt(gyro.getVelocityX()*gyro.getVelocityX() + gyro.getVelocityY()*gyro.getVelocityY());
  }

  public double getHeading(){
    return gyro.pidGet();
  }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("Velocity", getVelocity());
      SmartDashboard.putNumber("Heading", getHeading());
    }
}
