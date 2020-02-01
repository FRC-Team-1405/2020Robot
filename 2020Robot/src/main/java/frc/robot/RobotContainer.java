/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Autonomous1;
import frc.robot.commands.Autonomous2;
import frc.robot.commands.TestShooter;
import frc.robot.commands.TurnToAngle;
import frc.robot.sensors.ColorSensor;
import frc.robot.sensors.FMSData;
import frc.robot.sensors.LEDStrip;
import frc.robot.sensors.LIDARCanifier;
import frc.robot.subsystems.ArcadeDrive;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Autonomous1 auto1 = new Autonomous1();
  private final Autonomous2 auto2 = new Autonomous2();
  private final ArcadeDrive driveBase = new ArcadeDrive(); 
  private final Shooter launcher = new Shooter();
  private final ControlPanel controlPanel = new ControlPanel();
  // private final LEDStrip ledStrip = new LEDStrip(SPI.Port.kOnboardCS0, Constants.ledLength);
  private final LEDStrip ledStrip = new LEDStrip(9, 60);

  private final LIDARCanifier lidar = new LIDARCanifier(16);
  private final ColorSensor colorSensor = new ColorSensor(); 

  private XboxController driver = new XboxController(Constants.pilot);
  private XboxController operator = new XboxController(Constants.operator);
  private Intake intake = new Intake();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    driveBase.setDefaultCommand(
        new RunCommand(() -> driveBase.driveRobot(-driver.getY(GenericHID.Hand.kLeft),
                                                  driver.getX(GenericHID.Hand.kRight), true),
        driveBase));

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driver, XboxController.Button.kB.value)
      .whenPressed( new InstantCommand( driveBase::toggleDriveDirection, driveBase) ); 

    new JoystickButton(driver, XboxController.Button.kA.value) 
      .whenHeld( new RunCommand( colorSensor::readColor )); 

    // Testing
    new JoystickButton(driver, XboxController.Button.kStart.value)
      .whenPressed( new RunCommand( FMSData::getColor ));

    // new JoystickButton(driver, XboxController.Button.kY.value)
    //   .whenPressed( new InstantCommand( ledStrip::display ));
    // new JoystickButton(driver, XboxController.Button.kY.value)
    // .whenPressed( new InstantCommand( ledStrip::testOn ));
    new JoystickButton(driver, XboxController.Button.kY.value)
    .whenPressed( new InstantCommand( ledStrip::testLEDs ));
    
    new JoystickButton(driver, XboxController.Button.kBack.value)
      .whenHeld( new RunCommand( lidar::readDistance));

    // SmartDashboard.putNumber("Right/speed", 0); 
    // SmartDashboard.putNumber("Left/speed", 0); 
    // new JoystickButton(driver, XboxController.Button.kBumperRight.value)
    //   .whenHeld( new RunCommand( () -> { launcher.launch( SmartDashboard.getNumber("Left/speed", 0), SmartDashboard.getNumber("Right/speed", 0)); }) )
    //   .whenReleased( new InstantCommand( () -> { launcher.stop(); })); 
    new JoystickButton(driver, XboxController.Button.kBumperRight.value)
      .toggleWhenPressed( new TestShooter(launcher, driver::getPOV ) );

    DoubleSupplier setPoint = new DoubleSupplier(){
      public double getAsDouble() {
        return SmartDashboard.getNumber("TurnPID/setPoint", 0.0)+driveBase.getHeading();
      }
    };

    SmartDashboard.putNumber("TurnPID/offset", 15);
    new JoystickButton(driver, XboxController.Button.kX.value)
      .whenPressed( new InstantCommand( () -> {
        SmartDashboard.putNumber("TurnPID/setPoint", driveBase.getHeading()+SmartDashboard.getNumber("turnPID/offset", 15));
      }) )
      .whenHeld( new TurnToAngle(driveBase, () -> SmartDashboard.getNumber("TurnPID/setPoint", 0.0) ).beforeStarting( () -> {
                                  Constants.TurnPID.kP = SmartDashboard.getNumber("TurnPID/kP",Constants.TurnPID.kP) ;
                                  Constants.TurnPID.kI = SmartDashboard.getNumber("TurnPID/kI",Constants.TurnPID.kI) ;
                                  Constants.TurnPID.kD = SmartDashboard.getNumber("TurnPID/kD",Constants.TurnPID.kD) ;
                                })
      );

      new JoystickButton(operator, XboxController.Button.kX.value)
        .whenPressed( new FunctionalCommand( () -> controlPanel.rotationControl(Constants.ControlPanelConstants.ROTATION_CONTROL_DISTANCE),
                                              () -> {},
                                              (interrupted) -> { controlPanel.stop(); }, 
                                              controlPanel::isRotationComplete,
                                              controlPanel ) );

      new JoystickButton(operator, XboxController.Button.kB.value)
        .whenPressed(  new FunctionalCommand(controlPanel::positionControl,
                                              () -> {},
                                              (interrupted) -> { controlPanel.stop(); },
                                              controlPanel::checkColor,
                                              controlPanel
                                              )
                        .andThen( new FunctionalCommand( () -> controlPanel.rotationControl(Constants.ControlPanelConstants.COLOR_ADJUST),
                                              () -> {},
                                              (interrupted) -> { controlPanel.stop(); },
                                              controlPanel::isRotationComplete,
                                              controlPanel) ));

      new JoystickButton(operator, XboxController.Button.kBumperLeft.value)
        .whenPressed(  new FunctionalCommand( () -> controlPanel.rotationControl(-Constants.ControlPanelConstants.POSITION_ADJUST),
                                              () -> {},
                                              (interrupted) -> { controlPanel.stop(); },
                                              controlPanel::isRotationComplete,
                                              controlPanel));

      new JoystickButton(operator, XboxController.Button.kBumperRight.value)
      .whenPressed(  new FunctionalCommand( () -> controlPanel.rotationControl(Constants.ControlPanelConstants.POSITION_ADJUST),
                                            () -> {},
                                            (interrupted) -> { controlPanel.stop(); },
                                            controlPanel::isRotationComplete,
                                            controlPanel));
      new JoystickButton(driver, XboxController.Button.kBumperLeft.value)
      .whenHeld( new InstantCommand( intake :: enable))
      .whenReleased(new InstantCommand(intake :: disable));
                                                                   

  };


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return auto1;
  }
}
