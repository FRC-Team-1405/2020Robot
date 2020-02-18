/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;

import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArcadeDrive;

/**
 * Add your docs here.
 */
public class TrajectoryDrive {
    ArcadeDrive driveBase;
    Pose2d startPose;
    List<Translation2d> interiorWaypoints;
    Pose2d endPose;
    public TrajectoryDrive(ArcadeDrive driveBase, Pose2d startPose, List<Translation2d> interiorWaypoints, Pose2d endPose){
        this.driveBase = driveBase;
        this.startPose = startPose;
        this.interiorWaypoints = interiorWaypoints;
        this.endPose = endPose;
    }

    TrajectoryConfig config = new TrajectoryConfig(Constants.maxVelocity, Constants.maxAcceleration);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        startPose,
        // Pass through these two interior waypoints, making an 's' curve path
        interiorWaypoints,
        // End 3 meters straight ahead of where we started, facing forward
        endPose,
        // Pass config
        config
    );

    // RamseteCommand​(Trajectory trajectory, Supplier<Pose2d> pose, RamseteController follower, DifferentialDriveKinematics kinematics, 
    // BiConsumer<Double,​Double> outputMetersPerSecond, Subsystem... requirements)

    RamseteCommand ramseteCommand = new RamseteCommand(
            exampleTrajectory,
            driveBase::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            Constants.kDriveKinematics,
            driveBase::setVelocity,
            driveBase);

}
