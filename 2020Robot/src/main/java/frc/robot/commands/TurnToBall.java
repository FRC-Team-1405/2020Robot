/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.ArcadeDrive;

public class TurnToBall extends CommandBase {
  ArcadeDrive arcadeDrive;
  Limelight limelight;
  double speed;

  public TurnToBall(ArcadeDrive arcadeDrive, Limelight limelight, double speed) {
    this.arcadeDrive = arcadeDrive;
    this.speed = speed;
    this.limelight = limelight;

    addRequirements(arcadeDrive);
  }

  @Override
  public void initialize() {
    limelight.setPipeline((byte) 9);
    limelight.setLED((byte) 0);
  }

  @Override
  public void execute() {
    arcadeDrive.driveRobot(0, speed, false);
  }

  @Override
  public boolean isFinished() {
    if(limelight.hasTarget())
    {
      arcadeDrive.stop();
      return true;
    }
    return false;
  }
}
