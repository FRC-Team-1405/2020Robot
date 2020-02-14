/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.sensors.LEDStrip;
import frc.robot.sensors.LidarLitePWM;
import frc.robot.subsystems.ArcadeDrive;

public class ClimbLEDs extends CommandBase {

  private LEDStrip ledStrip;
  private ArcadeDrive driveBase;
  private DoubleSupplier leftDistance;
  private DoubleSupplier rightDistance;

  /**
   * Creates a new ClimbLEDs.
   */
  public ClimbLEDs(LEDStrip ledStrip, ArcadeDrive driveBase, DoubleSupplier leftDistance, DoubleSupplier rightDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ledStrip = ledStrip;
    this.driveBase = driveBase;
    this.leftDistance = leftDistance;
    this.rightDistance = rightDistance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //given robot weights, calculate neccesary robot distances
    // double leftRobotWeight = SmartDashboard.getNumber("Left_Robot_Weight", 200);
    // double rightRobotWeight = SmartDashboard.getNumber("Right_Robot_Weight", 200);
    // TODO implement algorithm for distances
    // double leftRobotDistance = 0.0;
    // double rightRobotDistance = 0.0;

    // For now use https://www.geogebra.org/m/aapcdzvf to find ideal distances and tolerances
    // Convert to centimeters
    double leftRobotDistance = SmartDashboard.getNumber("Left_Robot_Distance_Inches", 40)*2.54;
    double rightRobotDistance = SmartDashboard.getNumber("Right_Robot_Distance_Inches", 40)*2.54;

    double leftTolerance = SmartDashboard.getNumber("Left_Tolerance_Inches", 10)*2.54;
    double rightTolerance = SmartDashboard.getNumber("Right_Tolerance_Inches", 10)*2.54;
    
    // red leds -> stop
    // green -> move closer
    // yellow -> move farther
    if(Math.abs(leftRobotDistance - leftDistance.getAsDouble()) <= leftTolerance){
      ledStrip.red1();
    } else if(leftRobotDistance > leftDistance.getAsDouble()){
      ledStrip.green1();
    } else if(leftRobotDistance < leftDistance.getAsDouble()){
      ledStrip.blue1();
    }
    
    if(Math.abs(rightRobotDistance - rightDistance.getAsDouble()) <= rightTolerance){
      ledStrip.red2();
    } else if(rightRobotDistance > rightDistance.getAsDouble()){
      ledStrip.green2();
    } else if(rightRobotDistance < rightDistance.getAsDouble()){
      ledStrip.blue2();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
