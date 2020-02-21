/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.MathTools;
import frc.robot.sensors.LEDStrip;
import frc.robot.subsystems.Shooter;

public class LimelightLED extends CommandBase {
  private LEDStrip ledStrip;
  private Shooter shooter;
  private AddressableLEDBuffer addressableLEDBuffer;
  /**
   * Creates a new LimelightLED.
   */
  public LimelightLED(LEDStrip ledStrip, Shooter shooter) {
    this.ledStrip = ledStrip;
    this.shooter = shooter;
    this.addressableLEDBuffer = ledStrip.getLedBuffer() ;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double offset = shooter.limelight.getTX();
    double numberOfLeds = MathTools.map(offset, -29.8, 29.8, -Constants.UnderGlow.ledCount/2, Constants.UnderGlow.ledCount/2);
    if(numberOfLeds < 0){
      for(int i = 0; i < -numberOfLeds; i++){
        addressableLEDBuffer.setLED(Constants.UnderGlow.ledStart-i, Color.kGreen);
      }
    }else{
      for(int i = 0; i < Constants.UnderGlow.ledCount; i++){
        addressableLEDBuffer.setLED(i + Constants.UnderGlow.ledStart, Color.kGreen);
      }
    }


    ledStrip.displayLEDBuffer(addressableLEDBuffer);
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
