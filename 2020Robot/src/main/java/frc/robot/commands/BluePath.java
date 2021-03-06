/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.ArcadeDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class BluePath extends SequentialCommandGroup {

  public BluePath(Intake intake, ArcadeDrive arcadeDrive, Limelight limelight, Shooter shooter) {
    super(new ParallelCommandGroup(new TurnToBall(arcadeDrive, limelight, -0.1), new InstantCommand(shooter::setMaxElevation)),
          new ParallelCommandGroup(new InstantCommand(intake::enable), new DriveToBall(limelight, arcadeDrive)),
          new ParallelCommandGroup(new InstantCommand(intake::disable), new Indexer(shooter).withTimeout(0.5), new TurnToBall(arcadeDrive, limelight, 0.1)),
          new ParallelCommandGroup(new InstantCommand(intake::enable), new DriveToBall(limelight, arcadeDrive)),
          new ParallelCommandGroup(new InstantCommand(shooter::setElevation), new TurnToZero(arcadeDrive, 0.1)));
  }
}
