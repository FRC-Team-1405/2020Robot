/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArcadeDrive;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FireOnce extends SequentialCommandGroup {
  /**
   * Creates a new FireOnce.
   */
  public FireOnce(Shooter shooter, ArcadeDrive driveBase) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new FunctionalCommand(() -> {},
                                () -> { if(shooter.flywheelReady())
                                          shooter.fire();},
                                (interrupted) -> { shooter.stopIndexer();},
                                () -> {return false;} )  
          );
  }

  public FireOnce(Shooter shooter) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new FunctionalCommand(() -> {},
                                () -> { if(shooter.flywheelReady()){
                                          SmartDashboard.putBoolean("Shooter/FireActive", true);
                                          shooter.fire();
                                      }else{
                                        SmartDashboard.putBoolean("Shooter/FireActive", false);
                                        shooter.stopIndexer();
                                      }},
                                (interrupted) -> { shooter.stopIndexer();},
                                () -> {return false;} )  
          );
  }
}
 