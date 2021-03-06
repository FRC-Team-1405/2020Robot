/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.ArcadeDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class GalacticSearch extends SequentialCommandGroup {

  public GalacticSearch(Intake intake, ArcadeDrive arcadeDrive, Limelight limelight, Shooter shooter) {
    super(new InstantCommand(arcadeDrive::resetDistance),
          new DriveToBall(limelight, arcadeDrive),
          new CommandBase(){
            private CommandBase command;
            @Override
            public void initialize() {
             command = arcadeDrive.getDistance() > 5 ? new RedPath(intake, arcadeDrive, limelight, shooter) : new BluePath(intake, arcadeDrive, limelight, shooter);
            }
          
            @Override
            public void execute(){
              command.execute();
            }

            @Override
            public boolean isFinished() {
              return command.isFinished();
            }
          });
  }
}
/* 
onInit
  private Commmand executeMe = new A or new B
onExecute
  executeme.execute()
isFinished
  execureme.isFinishe()

*/