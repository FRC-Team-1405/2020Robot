/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */ 
  
   //This is a Falcon 500. Not sure if FX will work. 
   public WPI_TalonFX leftClimbMotor = new WPI_TalonFX(Constants.leftClimbMotor); 
   public WPI_TalonFX rightClimbMotor = new WPI_TalonFX(Constants.rightClimbMotor); 
  //  public CANSparkMax leftClimbMotor = new CANSparkMax(Constants.leftClimbMotor, MotorType.kBrushless);
  //  public CANSparkMax rightClimbMoter = new CANSparkMax(Constants.rightClimbMotor, MotorType.kBrushless);
   
  //Regular bois go here: 
  //  public WPI_TalonSRX buddyBarLiftMotorLeft = new WPI_TalonSRX(Constants.buddyBarLiftMotorLeft); 
  //  public WPI_TalonSRX buddyBarLiftMotorRight = new WPI_TalonSRX(Constants.buddyBarLiftMotorRight); 
  //  public WPI_TalonSRX frontClampMotorLeft = new WPI_TalonSRX(Constants.frontClampMotorLeft); 
  //  public WPI_TalonSRX backClampMotorLeft = new WPI_TalonSRX(Constants.backClampMotorLeft); 
  //  public WPI_TalonSRX frontClampMotorRight = new WPI_TalonSRX(Constants.frontClampMotorRight); 
  //  public WPI_TalonSRX backClampMotorRight = new WPI_TalonSRX(Constants.backClampMotorRight); 

  //Configurable values for the climb motor: 
  public double reachPosition = 0.0; 
  public double homePosition = 0.0;
  public boolean enabled = false; 

  DigitalInput leftFrontSwitch = new DigitalInput(Constants.leftFrontSwitchid);
  DigitalInput rightFrontSwitch = new DigitalInput(Constants.rightFrontSwitchid);
  DigitalInput leftBackSwitch = new DigitalInput(Constants.leftBackSwitchid);
  DigitalInput rightBackSwitch = new DigitalInput(Constants.rightBackSwitchid);

  public Climber() { 
    if(!Robot.fmsAttached){
      SmartDashboard.putNumber("Climb Position", reachPosition); 
      SmartDashboard.putNumber("Home Position", homePosition); 
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Climber Sensor Reading", leftFrontSwitch.get());
  } 

  public void toggleEnable(){
    enabled = !enabled;
  }

  public void directControl(DoubleSupplier left, DoubleSupplier right){
    double leftPos = leftClimbMotor.getSelectedSensorPosition() + left.getAsDouble();
    double rightPos = rightClimbMotor.getSelectedSensorPosition() + right.getAsDouble();
    leftClimbMotor.set(ControlMode.Position, leftPos);
    rightClimbMotor.set(ControlMode.Position, rightPos);
  }
  
  public void moveLeft(double distance){
    if(enabled){
      leftClimbMotor.set(ControlMode.Position, distance);
    }
  }

  public void moveRight(double distance){
    if(enabled){
      rightClimbMotor.set(ControlMode.Position, distance);
    }
  }

  public void reachUp(){  
    if(enabled){
      leftClimbMotor.set(ControlMode.Position, SmartDashboard.getNumber("Climb Position", reachPosition));
      rightClimbMotor.set(ControlMode.Position, SmartDashboard.getNumber("Climb Position", reachPosition));
    }
  } 

  public void goHome(){ 
    if(enabled){  
      leftClimbMotor.set(ControlMode.Position, SmartDashboard.getNumber("Home Position", homePosition));
      rightClimbMotor.set(ControlMode.Position, SmartDashboard.getNumber("Home Position", homePosition));
    }
  }

  boolean leftToggle = false;
  boolean rightToggle = false;
  boolean leftFrontOff = true;
  boolean leftBackOff = true;
  boolean rightFrontOff = true;
  boolean rightBackOff = true;
  public boolean isAligned(){
    if(leftFrontOff && !leftFrontSwitch.get()){
      leftToggle = !leftToggle;
      leftFrontOff = false;
    }
    if(leftBackOff && !leftBackSwitch.get()){
      leftToggle = !leftToggle;
      leftBackOff = false;
    }
    if(rightFrontOff && !rightFrontSwitch.get()){
      rightToggle = !rightToggle;
      rightFrontOff = false;
    }
    if(rightBackOff && !rightBackSwitch.get()){
      rightToggle = !rightToggle;
      rightBackOff = false;
    }
    if(leftFrontSwitch.get()){
      leftFrontOff = true;
    }
    if(leftBackSwitch.get()){
      leftBackOff = true;
    }
    if(rightFrontSwitch.get()){
      rightFrontOff = true;
    }
    if(rightBackSwitch.get()){
      rightBackOff = true;
    }
    return (rightToggle && leftToggle);
  }

  // get lengths in meters
  public double getLeftScissorLength(){
    return 0.0;
  }

  public double getRightScissorLength(){
    return 0.0;
  }
}
