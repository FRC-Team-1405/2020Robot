/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.SmartSupplier;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.ArcadeDrive;

public class DriveToBall extends CommandBase {
  
  private Limelight limelight;
  private ArcadeDrive arcadeDrive;
  private SmartSupplier speed = new SmartSupplier("Drive to ball/Rotation", 0.2);
  private SmartSupplier rotation = new SmartSupplier("Drive to ball/Speed", 0.1);

  //One secound of delay
  private double endDelay = 1000;
  private long start = System.currentTimeMillis();

  public DriveToBall(Limelight limelight, ArcadeDrive arcadeDrive) {  
    this.limelight = limelight;
    this.arcadeDrive = arcadeDrive;

    addRequirements(arcadeDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setPipeline((byte) 9);
    limelight.setLED((byte) 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!limelight.hasTarget())
    {
      arcadeDrive.driveRobot(speed.getAsDouble(), 0, false);
      return;
    }

    //Start trying to pickup the balls
    if(limelight.getTX() < 1){
      //Move left
      arcadeDrive.driveRobot(speed.getAsDouble(), -rotation.getAsDouble(), false);
    }else if(limelight.getTX() > 1){
      //Move right
      arcadeDrive.driveRobot(speed.getAsDouble(), rotation.getAsDouble(), false);
    }else{
      arcadeDrive.driveRobot(speed.getAsDouble(), 0, false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(limelight.hasTarget()){
        start = System.currentTimeMillis();
    }else{
        long finish = System.currentTimeMillis();
        if(finish - start >= endDelay)
            return true;
    }
    return false;
  }
}
