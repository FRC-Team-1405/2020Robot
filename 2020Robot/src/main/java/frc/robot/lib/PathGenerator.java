/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class PathGenerator {
    private static TrajectoryConfig config = new TrajectoryConfig(Constants.maxVelocity, Constants.maxAcceleration);

    static public Trajectory driveForward(){
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of( new Translation2d(1, 0)
                     ),
            new Pose2d(2, 0, new Rotation2d(0)),
            config
        );
    }

    static public Trajectory driveCurveTest(){
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of( new Translation2d(0.75, 0.5),
                     new Translation2d(1.50, 1.0),
                     new Translation2d(2.25, 0.5)
                     ),
            new Pose2d(3, 0, new Rotation2d(0)),
            config
        );
    }
}
