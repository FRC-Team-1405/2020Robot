package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.thirdcoast.swerve.SwerveDrive;
import frc.robot.lib.thirdcoast.swerve.SwerveDrive.DriveMode;
import frc.robot.lib.thirdcoast.swerve.SwerveDriveConfig;
import frc.robot.lib.thirdcoast.swerve.Wheel;

public class SwerveDriveBase extends SubsystemBase {

  private static final double DRIVE_SETPOINT_MAX = 0.0;
  private static final double ROBOT_LENGTH = 1.0;
  private static final double ROBOT_WIDTH = 1.0;
  private final SwerveDrive swerve = getSwerve();

  public SwerveDriveBase() {
  }

public void setDriveMode(DriveMode mode) {
    swerve.setDriveMode(mode);
  }

  public void stop() {
    swerve.stop();
  }

  public void zeroAzimuthEncoders() {
    swerve.zeroAzimuthEncoders();
  } 

  public void writeAzimuthPositions(){ 
    swerve.saveAzimuthPositions();
  }

  public void drive(double forward, double strafe, double azimuth, double speedLimit) {
    swerve.drive(forward, strafe, azimuth, speedLimit);
   }

  // Drive the robot in a straight line without using navX
  public void driveStraight(double speed, double turnspeed, double speedLimit){
    swerve.setFieldOriented(false);
    swerve.drive(speed, 0, turnspeed, speedLimit);
    swerve.setFieldOriented(true);
  }

  public void zeroGyro() {
    AHRS gyro = swerve.getGyro();
    gyro.setAngleAdjustment(0);
    double adj = (gyro.getAngle() % 360) + SwerveDriveConfig.gyroHardwareOffset;
    gyro.setAngleAdjustment(-adj);
  } 

  // Swerve configuration

  private SwerveDrive getSwerve() {
    SwerveDriveConfig config = new SwerveDriveConfig();
    config.wheels = getWheels();
    config.gyro = new AHRS(SPI.Port.kMXP);
    config.length = ROBOT_LENGTH;
    config.width = ROBOT_WIDTH;
    config.gyroLoggingEnabled = true;
    config.summarizeTalonErrors = false;

    return new SwerveDrive(config);
  }

  private Wheel[] getWheels() {
    // TalonSRXConfiguration azimuthConfig = new TalonSRXConfiguration();
    // azimuthConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    // azimuthConfig.continuousCurrentLimit = 10;
    // azimuthConfig.peakCurrentDuration = 0;
    // azimuthConfig.peakCurrentLimit = 0;
    // azimuthConfig.slot0.kP = 10.0;
    // azimuthConfig.slot0.kI = 0.0;
    // azimuthConfig.slot0.kD = 100.0;
    // azimuthConfig.slot0.kF = 0.0;
    // azimuthConfig.slot0.integralZone = 0;
    // azimuthConfig.slot0.allowableClosedloopError = 0;
    // azimuthConfig.motionAcceleration = 10_000;
    // azimuthConfig.motionCruiseVelocity = 800;

    // TalonSRXConfiguration driveConfig = new TalonSRXConfiguration();
    // driveConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    // driveConfig.continuousCurrentLimit = 40;
    // driveConfig.peakCurrentDuration = 0;
    // driveConfig.peakCurrentLimit = 0;

    // wheels should be created in the array as
    // [0] -> front_right
    // [1] -> front_left
    // [2] -> rear_left
    // [3] -> rear_right
    Wheel[] wheels = new Wheel[] {
      createWheel(Constants.SwerveBase.azimuthFrontRight, Constants.SwerveBase.driveFrontRight),
      createWheel(Constants.SwerveBase.azimuthFrontLeft, Constants.SwerveBase.driveFrontLeft),
      createWheel(Constants.SwerveBase.azimuthBackLeft, Constants.SwerveBase.driveBackLeft),
      createWheel(Constants.SwerveBase.azimuthBackRight, Constants.SwerveBase.driveBackRight)
    };
    
    return wheels;
  }

  private Wheel createWheel(int azimuthId, int driveId){
    TalonSRX azimuthTalon = new TalonSRX(azimuthId);
    // azimuthTalon.configAllSettings(azimuthConfig);

    TalonSRX driveTalon = new TalonSRX(driveId);
    // driveTalon.configAllSettings(driveConfig);
    // driveTalon.setNeutralMode(NeutralMode.Brake);

    return new Wheel(azimuthTalon, driveTalon, DRIVE_SETPOINT_MAX);
  }  

  @Override
  public void periodic() {  

    swerve.updateOdometry(); 
    
}
}