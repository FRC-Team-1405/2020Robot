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
    public final static int driveLeft = 3;
    public final static int driveRight = 1;
    public final static int driveLeftSlave1 = 4;
    public final static int driveRightSlave1 = 2;
    public final static int driveLeftSlave2 = 5;
    public final static int driveRightSlave2 = 6;

    public final static int controlPanel = 7;

    public final static int shooterLeft = 10;
    public final static int shooterRight = 11;

    public final static int pilot = 0;
    public final static int operator = 1;

    public static class TurnPID {
        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kF = 0.0;
    }
}
