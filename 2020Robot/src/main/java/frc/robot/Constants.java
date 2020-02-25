/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants { 
    public final static int PDP = 0;
    public final static int driveLeft = 1;
    public final static int driveRight = 2;
    public final static int driveLeftSlave = 3;
    public final static int driveRightSlave = 4;

    public final static int intakeTalon = 5;
    public final static int intakeDeploy = 6;
    public final static int indexerid = 7;
    public final static int triggerid = 8;
    public final static int shooterLeft = 9;
    public final static int shooterRight = 10; 
    public final static int turretid = 11;

    public final static int controlPanel = 12;

    public final static int leftScissor = 13;
    public final static int rightScissor = 14;

    public final static int leftActuatorId = 1;
    public final static int rightActuatorId = 2;

    public final static double deadBand = 0.033;

    //All climb/buddy climb motors here 
    // public final static int backClampMotorLeft = 12; 
    // public final static int frontClampMotorLeft = 13; 
    // public final static int frontClampMotorRight = 14; 
    // public final static int backClampMotorRight = 15;
    // public final static int buddyBarLiftMotorLeft = 16; 
    // public final static int buddyBarLiftMotorRight = 17; 
    public final static int leftClimbMotor = 18; 
    public final static int rightClimbMotor = 19;   

    public final static int leftFrontSwitchid = 0;
    public final static int rightFrontSwitchid = 1;
    public final static int leftBackSwitchid = 2;
    public final static int rightBackSwitchid = 3;

    //Led strip length 
    public final static int ledLength =20; 

    public final static int pilot = 0;
    public final static int operator = 1;

    public final static int lidarBufferSize = 20;

    public final static double auto1Speed = -0.25;
    public final static double auto1Distance = 0.75;//distance in meters

    // meters
    public final static double robotWidth = 6.9342;
    public final static double goalX = 2.404364;

    public class ShooterConstants {
        public final static int angleMin = -90;
        public final static int angleMax = 90;
        public final static int unitsMin = 12000;
        public final static int unitsMax = 24000;
        public final static double elevationMin = 0.2;
        public final static double elevationMax = 0.8;
        public final static double limelightError = 2.0;
        public final static int turretError = 100;
    }
    //Turret constants

    //TrajectoryDrive constants
    public static final double kTrackwidthMeters = 0.5461;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
	public static final double maxVelocity = 1;
	public static final double maxAcceleration = 0.5;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static int maxFlywheelError = 500;
    public static int closeFire = 1;
    public static int farFire = 4;
    public static int closeFire2 = 2;
    public static int farFire2 = 3;

    public static class TurnPID {
        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kF = 0.0;
    }

    public static class DriveStraightPID {
        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kF = 0.0;
    }

    public static class BalancePID {
        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kF = 0.0;
    }

    public static class Lidar {
        public final static int BAUD_RATE = 9600;
        public static final SerialPort.Port PORT = SerialPort.Port.kUSB1;
        public static final int DATA_BITS = 8;
        public static final SerialPort.Parity PARITY = SerialPort.Parity.kNone;
        public static final SerialPort.StopBits STOP_BITS = SerialPort.StopBits.kOne;
        public static final String LIDAR_KEY = "LIDAR VALUES";
    }

    public static class ControlPanelConstants {
        public final static int ROTATION_CONTROL_DISTANCE = 10000; 
        public final static int ROTATION_SEGMENT_DISTANCE = 417;
        public final static double SPEED = .3;
        public final static int COLOR_ADJUST = 1000;
        public final static int POSITION_ADJUST = 50;
        // TODO At some point, define a value that will set a motor rotation equivalent to turning one segment on the control panel (1'9/16")
    }

    public static class IntakeConstants {
        public final static int DEPLOY_POSITION = 1000;
        public final static int RETRACT_POSITION = 0;
        public final static int RENDEZVOUS_POSITION = 500; 
        public static double SPEED = -0.8;
    }

    public static class VelocityConversions{
        public final static double SensorTimePerSec = 10;
        public final static double WheelCircumference = (6*Math.PI);
        public final static double SensorUnitsPerRotation = 2048;
        public final static double DriveBaseReduction = 8.68;
        public final static double InchesPerMeter = 39.37;
        public final static double SensorToMeters = (1.0 / SensorUnitsPerRotation) * (1.0 / DriveBaseReduction) * WheelCircumference * (1.0 / InchesPerMeter);
        public final static double MetersToSensor = (1.0 / SensorToMeters);
        public final static double FVelocityToMetersPerSecond = (SensorTimePerSec * WheelCircumference)/(SensorUnitsPerRotation * DriveBaseReduction * InchesPerMeter); 
        public final static double MetersPerSecondToVelocity = 1.0/FVelocityToMetersPerSecond; 


    }
    public class VelocityPID{
        public final static double VelocityP = 0.5;
        public final static double VelocityI = 0;
        public final static double VelocityD = 0;
    }

    public class BatteryMonitor{
        public final static double maxVoltage = 12.5;
        public final static double minVoltage = 7;
        //ledCount has to be a multiple of 3
        public final static int ledStart = 0;
        public final static int ledStop = 15;
        public final static int ledCount = ledStop - ledStart;
        public final static int brightness = 100;
    }

    public class UnderGlow{
        //The under glow has 43 leds
        public final static int ledStart = 15;
        public final static int ledStop = 59;
        public final static int ledCount = ledStop - ledStart;
        public final static int brightness = 100;
    }

    public class PWM_Port{
        public final static int leds = 8;
        public final static int totalLEDCount = UnderGlow.ledCount + BatteryMonitor.ledCount;
    }
}
