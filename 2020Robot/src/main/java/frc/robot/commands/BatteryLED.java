/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.MathTools;
import frc.robot.sensors.LEDStrip;

public class BatteryLED extends CommandBase {
  private LEDStrip ledStrip;
  private AddressableLEDBuffer addressableLEDBuffer;

  public BatteryLED(LEDStrip ledStrip) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ledStrip = ledStrip;
    this.addressableLEDBuffer = ledStrip.getLedBuffer() ;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double voltage = RobotController.getBatteryVoltage();

    int numberOfLeds = (int)MathTools.map(voltage, Constants.BatteryMonitor.minVoltage, Constants.BatteryMonitor.maxVoltage, 0, Constants.BatteryMonitor.ledCount);


    for (var i = 0; i < Constants.BatteryMonitor.ledCount; i++) {
      if (i <= numberOfLeds){
        addressableLEDBuffer.setRGB(i, 0, 100, 25);
      }
      else{
        addressableLEDBuffer.setRGB(i, 0, 0, 0);
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
