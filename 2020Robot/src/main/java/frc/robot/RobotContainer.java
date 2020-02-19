/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.BatteryMonitor;
import frc.robot.commands.Autonomous1;
import frc.robot.commands.Autonomous2;
import frc.robot.commands.BatteryLED;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DriveByVelocity;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.Fire;
import frc.robot.commands.ClimbLEDs;
import frc.robot.commands.TestShooter;
import frc.robot.commands.TurnToAngle;
import frc.robot.sensors.ColorSensor;
import frc.robot.sensors.FMSData;
import frc.robot.sensors.LEDStrip;
import frc.robot.sensors.LIDARCanifier;
import frc.robot.sensors.LidarLitePWM;
import frc.robot.sensors.LidarReader;
import frc.robot.subsystems.ArcadeDrive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final ArcadeDrive driveBase = new ArcadeDrive();
  private final Shooter launcher = new Shooter();
  private Intake intake = new Intake();
  private final Climber climber = new Climber();
  private final ControlPanel controlPanel = new ControlPanel();
  //private final LEDStrip ledStrip = new LEDStrip(3, 300);
  private final LIDARCanifier lidar = new LIDARCanifier(16);
  private final LidarLitePWM leftLidar = new LidarLitePWM(new DigitalInput(10));
  private final LidarLitePWM rightLidar = new LidarLitePWM(new DigitalInput(11));
  // private final LidarReader lidarReader = new LidarReader();
  private final ColorSensor colorSensor = new ColorSensor();

  private XboxController driver = new XboxController(Constants.pilot);
  private XboxController operator = new XboxController(Constants.operator);

  private final Autonomous1 auto1 = new Autonomous1(driveBase);
  private final Autonomous2 auto2 = new Autonomous2(driveBase, launcher);

  public final BatteryLED batteryMonitor = new BatteryLED( new LEDStrip(Constants.PWM_Port.batteryDisplay, Constants.BatteryMonitor.ledCount));


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    initShuffleBoard();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    driveBase.setDefaultCommand( new DefaultDrive( this::driveSpeed, this::driveRotation, driveBase) );

    climber.setDefaultCommand( new RunCommand( () -> {
      climber.directControl( -operator.getY(Hand.kLeft), operator.getY(Hand.kRight) );
    }, climber));

    // lidarReader.start();
  }

  SlewRateLimiter driveSpeedFilter = new SlewRateLimiter(0.5);
  private double driveSpeed(){
    double speed = -driver.getY(Hand.kLeft);
    if(Math.abs(speed) < Constants.deadBand)
      speed = 0.0;
    SmartDashboard.putNumber("Drive_Speed", speed);
    return driveSpeedFilter.calculate(speed);
  }

  SlewRateLimiter driveRotationFilter = new SlewRateLimiter(0.5);
  private double driveRotation(){
    double rotation = driver.getX(Hand.kRight);
    if(Math.abs(rotation) < Constants.deadBand)
      rotation = 0.0;
    SmartDashboard.putNumber("Drive_Rotation", rotation);
    return driveRotationFilter.calculate(rotation);
  }

  SendableChooser<Integer> autoSelector; 
   
  private void initShuffleBoard(){
    autoSelector = new SendableChooser<Integer>();
    autoSelector.addOption("Do nothing", 0);
    autoSelector.addOption("Drive forward.", 1);
    autoSelector.addOption("Shoot then drive", 2);
    autoSelector.addOption("Auto 3", 3);
    autoSelector.setDefaultOption("Do nothing", 0);

    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto") ;
    autoTab.add(autoSelector)
            .withWidget(BuiltInWidgets.kComboBoxChooser);
    SmartDashboard.putNumber("Auto/Selected_Auto", 1);
    autoTab.add("Auto/Initial_Delay", 0); 

  

    ShuffleboardTab testCommandsTab = Shuffleboard.getTab("Test Commands"); 
    testCommandsTab.add( new TestShooter(launcher, driver::getPOV));
    testCommandsTab.add( new Fire(launcher));
    //testCommandsTab.add(new ClimbLEDs(ledStrip, driveBase, leftLidar::getDistance, rightLidar::getDistance));
    testCommandsTab.add( new DriveByVelocity(driveBase));
    RunCommand getColor = new RunCommand( FMSData::getColor );
    getColor.setName("Get_Color");
    testCommandsTab.add(getColor);
    testCommandsTab.add( batteryMonitor );
    //InstantCommand displayLEDs = new InstantCommand(ledStrip::testLEDs);
    //displayLEDs.setName("Display_LEDs");
    //testCommandsTab.add(displayLEDs);
    RunCommand readDistance = new RunCommand(lidar::readDistance);
    readDistance.setName("Read_Distance");
    testCommandsTab.add(readDistance);
    RunCommand readColor = new RunCommand(colorSensor::readColor);
    readColor.setName("Read_Color");
    testCommandsTab.add(readColor);

    // SmartDashboard.putNumber("Left_Robot_Weight", 200);
    // SmartDashboard.putNumber("Right_Robot_Weight", 200);

    // For now use https://www.geogebra.org/m/aapcdzvf to find ideal distances and tolerances
    SmartDashboard.putNumber("Left_Robot_Distance_Inches", 40);
    SmartDashboard.putNumber("Right_Robot_Distance_Inches", 40);
    SmartDashboard.putNumber("Left_Tolerance_Inches", 10);
    SmartDashboard.putNumber("Right_Tolerance_Inches", 10);

    //SmartDashboard.putData( new PowerDistributionPanel(Constants.PDP) );
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

    new JoystickButton(driver, XboxController.Button.kBumperLeft.value)
      .whenHeld( new InstantCommand( intake :: enable))
      .whenReleased(new InstantCommand(intake :: disable)); 

    SmartDashboard.putNumber("Right/speed", 0); 
    SmartDashboard.putNumber("Left/speed", 0); 
    new JoystickButton(driver, XboxController.Button.kBumperRight.value)
      .whenHeld( new RunCommand( () -> { launcher.launch( SmartDashboard.getNumber("Left/speed", 0), SmartDashboard.getNumber("Right/speed", 0)); }) )
      .whenReleased( new InstantCommand( () -> { launcher.stopFlywheels(); })); 
    // new JoystickButton(driver, XboxController.Button.kBumperRight.value)
    //   .whenHeld( new RunCommand( () -> { launcher.fire(); }) )
    //   .whenReleased( new InstantCommand( () -> { launcher.stop(); })); 
      
    // new JoystickButton(driver, XboxController.Button.kBumperRight.value)
    //   .toggleWhenPressed( new TestShooter(launcher, driver::getPOV ) );

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
  };


  // An example selector method for the selectcommand.  Returns the selector that will select
  // which command to run.  Can base this choice on logical conditions evaluated at runtime.
  
  private int select() {
    return (int) autoSelector.getSelected();
  }

  // An example selectcommand.  Will select from the three commands based on the value returned
  // by the selector method at runtime.  Note that selectcommand works on Object(), so the
  // selector does not have to be an enum; it could be any desired type (string, integer,
  // boolean, double...)
  private final Command selectCommand =
      new SelectCommand(
          // Maps selector values to commands
          Map.ofEntries(
              Map.entry(0, new PrintCommand("Do nothing")),
              Map.entry(1, auto1),
              Map.entry(2, auto2),
              Map.entry(3, new PrintCommand("Command three was selected!"))
          ),
          this::select
      );

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoCommand;
    return selectCommand;
  }
}
