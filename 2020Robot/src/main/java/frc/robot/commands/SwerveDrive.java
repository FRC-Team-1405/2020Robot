/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDriveBase;


public final class SwerveDrive extends CommandBase {
  private static final double DEADBAND = 0.05;

  private SwerveDriveBase driveBase;
  private DoubleSupplier getForward;
  private DoubleSupplier getStrafe;
  private DoubleSupplier getYaw;

  public SwerveDrive(DoubleSupplier getForward, DoubleSupplier getStrafe, DoubleSupplier getYaw, SwerveDriveBase driveBase) {
    this.getForward = getForward;
    this.getStrafe = getStrafe;
    this.getYaw = getYaw;
    this.driveBase = driveBase;
    addRequirements(driveBase);
  }

  @Override
  public void initialize() {
    // driveBase.setDriveMode(TELEOP);
  }

  @Override
  public void execute() {
    double forward = deadband(getForward.getAsDouble());
    double strafe = deadband(getStrafe.getAsDouble());
    double azimuth = deadband(getYaw.getAsDouble());

    driveBase.drive(forward, strafe, azimuth);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveBase.drive(0.0, 0.0, 0.0);
  }

  private double deadband(double value) {
    if (Math.abs(value) < DEADBAND) return 0.0;
    return value;
  }
}
