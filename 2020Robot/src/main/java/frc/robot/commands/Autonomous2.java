/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArcadeDrive;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Autonomous2 extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous2.
   */
  public Autonomous2(ArcadeDrive driveBase, Shooter shooter) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    // super( new WaitCommand(SmartDashboard.getNumber("Auto/Initial_Delay", 0)), new FireOnce(shooter, driveBase).withTimeout(12  - SmartDashboard.getNumber("Auto/Initial_Delay", 0)),
    // new DriveDistance(driveBase, Constants.auto1Distance, Constants.auto1Speed));

    super( new InstantCommand(() -> {shooter.prepFlywheels(10000, 10000);}),
          new WaitCommand(SmartDashboard.getNumber("Auto/Initial_Delay", 0)),
          new FireOnce(shooter).raceWith(new WaitCommand(5)),
          // new InstantCommand(shooter::fire),
          new InstantCommand(() -> {shooter.stopFlywheels(); shooter.stopIndexer();}),
          new DriveDistance(driveBase, Constants.auto1Distance, Constants.auto1Speed));
  }
}
