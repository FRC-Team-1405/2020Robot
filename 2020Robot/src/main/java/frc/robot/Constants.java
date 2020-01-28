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

    public static class TurnPID {
        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kF = 0.0;
    }

    public static class ControlPanelConstants {
        public final static int ROTATION_DISTANCE = 10000 ; 
        public final static double SPEED = .3;
        public final static int COLOR_ADJUST = 1000;
        public final static int POSITION_ADJUST = 50;
    }

    public static class IntakeConstants{
        public final static int DEPLOY_POSITION = 1000;
        public final static int RETRACT_POSITION = 0;
        public static double SPEED = .8;
    }
}
