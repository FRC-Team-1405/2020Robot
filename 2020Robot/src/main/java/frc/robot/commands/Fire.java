/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class Fire extends CommandBase {
  /**
   * Creates a new Fire.
   */
  private Shooter shooter;
  public Fire(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    addRequirements(this.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.limelight.setPipeline((byte) 2);
    shooter.limelight.setLED((byte) 3);
    shooter.prepFlywheels(shooter.limelight.getTX());
    shooter.turnTurret((int) shooter.limelight.getTX());
    shooter.stopIndexer();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.turretReady() && shooter.flywheelReady()){
      shooter.fire();
    }else{
      shooter.stopIndexer();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.limelight.setPipeline((byte) 3);
    shooter.limelight.setPipeline((byte) 1);
    shooter.stopIndexer();
    shooter.stopFlywheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; 
  }
}
