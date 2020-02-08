/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static int driveLeft = 1;
    public final static int driveRight = 2;
    public final static int driveLeftSlave = 3;
    public final static int driveRightSlave = 4;

    public final static int intakeTalon = 5;
    public final static int intakeDeploy = 6;

    public final static int controlPanel = 7;

    public final static int shooterLeft = 10;
    public final static int shooterRight = 11; 

    //All climb/buddy climb motors here 
    public final static int backClampMotorLeft = 12; 
    public final static int frontClampMotorLeft = 13; 
    public final static int frontClampMotorRight = 14; 
    public final static int backClampMotorRight = 15;
    public final static int buddyBarLiftMotorLeft = 16; 
    public final static int buddyBarLiftMotorRight = 17; 
    public final static int leftClimbMotor = 18; 
    public final static int rightClimbMotor = 19;   

    //Led strip length 
    public final static int ledLength =29; 

    public final static int pilot = 0;
    public final static int operator = 1;

    public final static int lidarBufferSize = 20;

    public final static double auto1Speed = 0.5;
    public final static double auto1Distance = 20000;

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
        public static double SPEED = .8;
    }

    public static class VelocityConversions{
        public final static double SensorUnitsForMilliseconds = (1/100);
        public final static int MillisecondsToSensorUnits = (100/1);
        public final static double SecondsUnitsForMilliseconds = (1/1000);
        public final static int MillisecondsToSeconds = (1000/1);
        public final static double FRotationsToSensorUnits = (1/2048);
        public final static int SensorUnitsToFRotations = (2048/1);
        public final static double DriveShaftRotationsToFRotations = (1/8.68);
        public final static double FRotationsToDriveShaftRotations = (8.68/1);
        public final static double InchesToDriveShaftRotations = ((6*Math.PI)/1);
        public final static double DriveShaftRotationsToInches = (1/(6*Math.PI));
        public final static double MetersToInches = (1/39.37);
        public final static double InchesToMeters = (39.37/1);
        public final static double TimeAndFalconVToDriveV = (SensorUnitsForMilliseconds * MillisecondsToSeconds);
        public final static double DriveVToTimeAndFalconV = (MillisecondsToSeconds * SensorUnitsForMilliseconds);
        public final static double FalconVToDriveV = (SensorUnitsForMilliseconds * FRotationsToSensorUnits * DriveShaftRotationsToFRotations * InchesToDriveShaftRotations * MetersToInches);
        public final static double DriveVToFalconV = (MetersToInches * InchesToDriveShaftRotations * DriveShaftRotationsToFRotations * FRotationsToSensorUnits * SensorUnitsForMilliseconds);
    }
}
