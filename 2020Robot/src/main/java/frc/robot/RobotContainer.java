/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.commands.FireOnce;
import frc.robot.commands.FollowPath;
import frc.robot.commands.TestShooter;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.UnderGlow;
import frc.robot.lib.MathTools;
import frc.robot.lib.PathGenerator;
import frc.robot.lib.SmartSupplier;
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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  private final LIDARCanifier lidar = new LIDARCanifier(16);
  private final LidarLitePWM leftLidar = new LidarLitePWM(new DigitalInput(10));
  private final LidarLitePWM rightLidar = new LidarLitePWM(new DigitalInput(11));
  // private final LidarReader lidarReader = new LidarReader();
  private final ColorSensor colorSensor = new ColorSensor();

  private XboxController driver = new XboxController(Constants.pilot);
  private XboxController operator = new XboxController(Constants.operator);

  private final Autonomous1 auto1 = new Autonomous1(driveBase);
  private final Autonomous2 auto2 = new Autonomous2(driveBase, launcher);

  private LEDStrip ledStrip = new LEDStrip(Constants.PWM_Port.leds, Constants.PWM_Port.totalLEDCount);
  public final UnderGlow underGlow = new  UnderGlow(ledStrip);
  public final BatteryLED batteryMonitor = new BatteryLED(ledStrip);

  private SmartSupplier lowLeft = new SmartSupplier("Shooter/Low/Left", 10000);
  private SmartSupplier lowRight = new SmartSupplier("Shooter/Low/Right", 10000);
  private SmartSupplier midLeft = new SmartSupplier("Shooter/Mid/Left", 11000);
  private SmartSupplier midRight = new SmartSupplier("Shooter/Mid/Right", 11000);
  private SmartSupplier highLeft = new SmartSupplier("Shooter/High/Left", 16000);
  private SmartSupplier highRight = new SmartSupplier("Shooter/High/Right", 16000);
  public static double increase = 0;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    initShuffleBoard();
    launcher.limelight.setPipeline((byte) 0);
    launcher.limelight.setLED((byte) 1);

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    driveBase.setDefaultCommand( new DefaultDrive( this::driveSpeed, this::driveRotation, driveBase) );

    climber.setDefaultCommand( new RunCommand( () -> {
      climber.directControl( -operator.getY(Hand.kLeft), operator.getY(Hand.kRight) );
    }, climber));

    // climber.setDefaultCommand( new RunCommand( () -> {
    //   climber.positionControl( this::leftScissorPos, this::rightScissorPos );
    // }, climber));

    // lidarReader.start();
  }

  SlewRateLimiter driveSpeedFilter = new SlewRateLimiter(0.5);
  private double driveSpeed(){
    double speed = -driver.getY(Hand.kLeft);
    if(Math.abs(speed) < Constants.deadBand)
      speed = 0.0;
    // SmartDashboard.putNumber("Drive_Speed", speed);
    // return driveSpeedFilter.calculate(speed);
    return speed;
  }

  SlewRateLimiter driveRotationFilter = new SlewRateLimiter(0.5);
  private double driveRotation(){
    double rotation = driver.getX(Hand.kRight);
    if(Math.abs(rotation) < Constants.deadBand)
      rotation = 0.0;
    // SmartDashboard.putNumber("Drive_Rotation", rotation);
    // return driveRotationFilter.calculate(rotation);
    return rotation;
  }


  private double leftScissorPos(){
    double pos = -operator.getY(Hand.kLeft);
    if(Math.abs(pos) < Constants.deadBand)
      pos = 0.0;
    return MathTools.map(pos, -1.0, 1.0, -10.0, 10.0);
  }

  private double rightScissorPos(){
    double pos = -operator.getY(Hand.kRight);
    if(Math.abs(pos) < Constants.deadBand)
    pos = 0.0;
    return MathTools.map(pos, -1.0, 1.0, -10.0, 10.0);
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

  
    if(!Robot.fmsAttached){
      ShuffleboardTab testCommandsTab = Shuffleboard.getTab("Test Commands"); 
      testCommandsTab.add( new TestShooter(launcher, driver::getPOV));
      testCommandsTab.add( new FireOnce(launcher)
                      .andThen(new InstantCommand( () -> {launcher.stopFlywheels(); launcher.stopIndexer();}) ));
      testCommandsTab.add( new DriveByVelocity(driveBase));

      RunCommand getColor = new RunCommand( FMSData::getColor );
      getColor.setName("Get_Color");
      testCommandsTab.add(getColor);
  
      testCommandsTab.add( batteryMonitor );
      testCommandsTab.add( underGlow );
     
      RunCommand readDistance = new RunCommand(lidar::readDistance);
      readDistance.setName("Read_Distance");
      testCommandsTab.add(readDistance);

      RunCommand turnTurret = new RunCommand(launcher::turnTurret);
      turnTurret.setName("Turn_Turret");
      testCommandsTab.add(turnTurret);
  
      RunCommand readColor = new RunCommand(colorSensor::readColor);
      readColor.setName("Read_Color");
      testCommandsTab.add(readColor);
  
      InstantCommand resetPosition = new InstantCommand( () -> { driveBase.resetPosition(); } );
      resetPosition.setName("Reset Position");
      testCommandsTab.add(resetPosition);
  
      InstantCommand stopFlywheels = new InstantCommand( launcher::stopFlywheels );
      stopFlywheels.setName("Stop Flywheels");
      testCommandsTab.add(stopFlywheels);
  
      SmartDashboard.putNumber("Shooter/Elevation", 0);
      InstantCommand setElevation = new InstantCommand( () -> {launcher.setElevationManual(SmartDashboard.getNumber("Shooter/Elevation", 0)); });
      setElevation.setName("Set Elevation");
      testCommandsTab.add(setElevation);
  
    }

     SmartDashboard.putData( new PowerDistributionPanel(Constants.PDP) );
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /**
     * Driver:
     * +Left joystick: drive 
     * +Right joystick: turn
     * +B: drive backwards
     * +Right bumper: intake
     * +Left bumper: outtake
     * +X: toggle intake elevation
     * +D-pad up: increase shooter power
     * +D-pad down: decrease shooter power
     * 
     * Operator:
     * +Right bumper: run indexer
     * +Left bumper: shoot
     * +Y: prep flywheels auto
     * +B: prep flywheels close
     * +A: prep flywheels far
     * +X: stop flywheels
     * +Left trigger: manual turret adjust left
     * +Right trigger: manual turret adjust right
     * +D-pad left: toggle limelight pipeline
     * +D-pad right toggle shooter elevation
     * +D-pad up: scissors up
     * +D-pad down: scissors down
     * +Left joystick: left scissor
     * +Right joystick: right scissor
     * +Start: scissors enable
     */

     //B: drive backwards
    new JoystickButton(driver, XboxController.Button.kB.value)
      .whenPressed( new InstantCommand( driveBase::toggleDriveDirection, driveBase) ); 

    //Right bumper: intake
    new JoystickButton(driver, XboxController.Button.kBumperRight.value)
      .whenHeld( new InstantCommand( intake :: enable))
      .whenReleased( new InstantCommand(intake :: disable)); 
    
    //Left bumper: outtake 
    new JoystickButton(driver, XboxController.Button.kBumperLeft.value)
      .whenHeld( new InstantCommand( intake::outtake))
      .whenHeld( new InstantCommand( launcher::outtake))
      .whenReleased( new InstantCommand(intake::disable))
      .whenReleased( new InstantCommand(launcher::stopIndexer));

    //X toggle intake elevation
    new JoystickButton(driver, XboxController.Button.kX.value)
      .whenPressed( new FunctionalCommand(intake::toggleElevation,
                                          () -> {},
                                          (interrupted) -> {intake.stop();}, 
                                          intake::armInPosition,
                                          intake ) );

    //D-pad up: increase power
    new POVButton(driver, 0)
      .whenPressed( new InstantCommand( () -> {increase += 250;} ));

    //D-pad down: decrease power
    new POVButton(driver, 0)
      .whenPressed( new InstantCommand( () -> {increase -= 250;} ));




    //Left bumper: fire auto
    new JoystickButton(operator, XboxController.Button.kBumperLeft.value)
      .whenHeld( new FireOnce(launcher, driveBase) )
      .whenReleased(launcher::stopIndexer);

    //Right bumper: run indexer
    new JoystickButton(operator, XboxController.Button.kBumperRight.value)
      .whenPressed(launcher::fire)
      .whenReleased(launcher::stopIndexer);

    //Y: prep flywheels auto
    new JoystickButton(operator, XboxController.Button.kY.value)
      .whenPressed( new InstantCommand(() -> {launcher.prepFlywheels(lowLeft, lowRight); }));

    //B: prep flywheels close
    new JoystickButton(operator, XboxController.Button.kB.value)
      .whenPressed( new InstantCommand(() -> {launcher.prepFlywheels(midLeft, midRight); }));

    //A: prep flywheels far
    new JoystickButton(operator, XboxController.Button.kA.value)
      .whenPressed( new InstantCommand(() -> {launcher.prepFlywheels(highLeft, highRight); }));

    //X: stop flywheels
    new JoystickButton(operator, XboxController.Button.kX.value)
      .whenPressed( new InstantCommand(launcher::stopFlywheels))
      .whenPressed( new InstantCommand(launcher::goHome));

    //Left trigger: manual turret adjust left
    new Trigger(() -> operator.getTriggerAxis(Hand.kLeft) > 0.2 )
    .whileActiveOnce( new RunCommand( () -> {launcher.tracking = false;
                                            if(launcher.turretTurnIsComplete())
                                              launcher.turnTurret((int) -MathTools.map(operator.getTriggerAxis(Hand.kLeft), 0.2, 1, 1, 10));
                                            }) );
    
  //Right trigger: manual turret adjust right
  new Trigger(() -> operator.getTriggerAxis(Hand.kRight) > 0.2 )
    .whileActiveOnce( new RunCommand( () -> {launcher.tracking = false;
                                              if(launcher.turretTurnIsComplete())
                                              launcher.turnTurret((int) MathTools.map(operator.getTriggerAxis(Hand.kRight), 0.2, 1, 1, 10));
                                            }) );

    // //Left trigger: rotation control
    // new Trigger(() -> operator.getTriggerAxis(Hand.kLeft) > 0.1 )
    //   .whileActiveOnce( new FunctionalCommand( () -> controlPanel.rotationControl(Constants.ControlPanelConstants.ROTATION_CONTROL_DISTANCE),
    //                                         () -> {},
    //                                         (interrupted) -> { controlPanel.stop(); }, 
    //                                         controlPanel::isRotationComplete,
    //                                         controlPanel ) );

    // //Right trigger: position control
    // new Trigger(() -> operator.getTriggerAxis(Hand.kRight) > 0.1 )
    //   .whileActiveOnce(  new FunctionalCommand(controlPanel::positionControl,
    //                                       () -> {},
    //                                       (interrupted) -> { controlPanel.stop(); },
    //                                       controlPanel::checkColor,
    //                                       controlPanel
    //                                       )
    //                 .andThen( new FunctionalCommand( () -> controlPanel.rotationControl(Constants.ControlPanelConstants.COLOR_ADJUST),
    //                                       () -> {},
    //                                       (interrupted) -> { controlPanel.stop(); },
    //                                       controlPanel::isRotationComplete,
    //                                       controlPanel) ));

    // //D-pad left: control panel left
    // new POVButton(operator, 270)
    // .whenPressed(  new FunctionalCommand( () -> controlPanel.rotationControl(-Constants.ControlPanelConstants.POSITION_ADJUST),
    //                                     () -> {},
    //                                     (interrupted) -> { controlPanel.stop(); },
    //                                     controlPanel::isRotationComplete,
    //                                     controlPanel));   

    // //D-pad right: control panel right
    // new POVButton(operator, 90)
    // .whenPressed(  new FunctionalCommand( () -> controlPanel.rotationControl(Constants.ControlPanelConstants.POSITION_ADJUST),
    //                                       () -> {},
    //                                       (interrupted) -> { controlPanel.stop(); },
    //                                       controlPanel::isRotationComplete,
    //                                       controlPanel)); 

    //D-Pad left: toggle limelight pipeline
    new POVButton(operator, 270)
    .whenPressed( new InstantCommand(launcher::togglePipeline));

    //D-pad right: toggle shooter elevation
    new POVButton(operator, 90)
    .whenPressed(new InstantCommand( launcher::toggleElevation, launcher));

    //D-pad up: scissors up
    new POVButton(operator, 0)
      .whenPressed( new InstantCommand( climber::reachUp ));

    //D-pad down: scissors down
    new POVButton(operator, 180)
      .whenPressed( new InstantCommand( climber::goHome ));

    //Start: enable scissors
    new JoystickButton(operator, XboxController.Button.kStart.value)
      .whenPressed( new InstantCommand( climber::toggleEnable));

                                          
    // DoubleSupplier setPoint = new DoubleSupplier(){
    //   public double getAsDouble() {
    //     return SmartDashboard.getNumber("TurnPID/setPoint", 0.0)+driveBase.getHeading();
    //   }
    // };

    // SmartDashboard.putNumber("TurnPID/offset", 15);
    // new JoystickButton(driver, XboxController.Button.kX.value)
    //   .whenPressed( new InstantCommand( () -> {
    //     SmartDashboard.putNumber("TurnPID/setPoint", driveBase.getHeading()+SmartDashboard.getNumber("turnPID/offset", 15));
    //   }) )
    //   .whenHeld( new TurnToAngle(driveBase, () -> SmartDashboard.getNumber("TurnPID/setPoint", 0.0) ).beforeStarting( () -> {
    //                               Constants.TurnPID.kP = SmartDashboard.getNumber("TurnPID/kP",Constants.TurnPID.kP) ;
    //                               Constants.TurnPID.kI = SmartDashboard.getNumber("TurnPID/kI",Constants.TurnPID.kI) ;
    //                               Constants.TurnPID.kD = SmartDashboard.getNumber("TurnPID/kD",Constants.TurnPID.kD) ;
    //                             })
    //   );

    //   new JoystickButton(driver, XboxController.Button.kStart.value)
    //     .whenHeld( new FollowPath( PathGenerator.driveCurveTest(), driveBase).andThen( () -> driveBase.stop() ) )
    //     .whenReleased( () -> driveBase.stop() );

    //   new JoystickButton(driver, XboxController.Button.kA.value) 
    //   .whenHeld(new DriveByVelocity(driveBase));                         
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
