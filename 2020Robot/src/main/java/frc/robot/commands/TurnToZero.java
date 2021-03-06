/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArcadeDrive;

public class TurnToZero extends CommandBase {
  ArcadeDrive arcadeDrive;
  double speed;
  double angle;

  public TurnToZero(ArcadeDrive arcadeDrive, double speed) {
    this.arcadeDrive = arcadeDrive;
    this.speed = speed;

    addRequirements(arcadeDrive);
  }

  @Override
  public void execute() {
    if(arcadeDrive.getHeading() <= 180){
      arcadeDrive.driveRobot(0, -speed, false);
    }
    else{
      arcadeDrive.driveRobot(0, speed, false);
    }

    angle = arcadeDrive.getHeading();
  }

  @Override
  public boolean isFinished() {
    if(angle < 1 || angle > 1)
    {
      return true;
    }
    else{
      return false;
    }    
  }
}
