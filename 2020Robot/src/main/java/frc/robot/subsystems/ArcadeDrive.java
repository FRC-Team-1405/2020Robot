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
  
  WPI_TalonSRX driveLeft = new WPI_TalonSRX(Constants.driveLeft);
  WPI_TalonSRX driveRight = new WPI_TalonSRX(Constants.driveRight);
  WPI_TalonSRX driveLeftSlave = new WPI_TalonSRX(Constants.driveLeftSlave);
  WPI_TalonSRX driveRightSlave = new WPI_TalonSRX(Constants.driveRightSlave);
  DifferentialDrive driveBase = new DifferentialDrive(driveLeft, driveRight);

  private AHRS gyro = new AHRS(I2C.Port.kMXP);
  boolean driveForward = true;
  
  public ArcadeDrive() {
    driveLeft.configNeutralDeadband(0.04, 10);
    driveRight.configNeutralDeadband(0.04, 10);

    driveLeft.set(ControlMode.PercentOutput, 0);
    driveRight.set(ControlMode.PercentOutput, 0);
    driveLeftSlave.set(ControlMode.Follower, Constants.driveLeft);
    driveRightSlave.set(ControlMode.Follower, Constants.driveRight);

    driveBase.setDeadband(0.0);
  }

  public void toggleDriveDirection(){
    driveForward = !driveForward;
  }

  public void driveRobot(double xSpeed, double zRotation, boolean squareInputs){
    System.out.printf("driveRobot %f %f\n", xSpeed, zRotation);
    if(driveForward){
      driveBase.arcadeDrive(xSpeed, zRotation, squareInputs);
    }else{
      driveBase.arcadeDrive(-xSpeed, zRotation, squareInputs);
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