/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Trajectories;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.NeutrinoTrajectoryConfigs;

/**
 * This is the example trajectory from wpilib
 */
public class ExampleTrajectory
{
    public static List<Pose2d> sixBall = new ArrayList<Pose2d>();

    public ExampleTrajectory()
    {
        sixBall.add(new Pose2d(0, 0, new Rotation2d(0)));
        sixBall.add(new Pose2d(4.6, 0, new Rotation2d(0)));
    }

    public static final Trajectory exampleTraj = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        NeutrinoTrajectoryConfigs.m_DefaultConfig);

    public static final Trajectory sixBall0 = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(4.6, 0, new Rotation2d(0))),
        NeutrinoTrajectoryConfigs.m_DefaultConfig);

    public static final Trajectory sixBall1 = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(4.6, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(2, 0), new Translation2d(1, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass config
        NeutrinoTrajectoryConfigs.m_ReverseConfig);

    public static final Trajectory eightBall0 = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(2.9, -2.1, Rotation2d.fromDegrees(-60))),
        NeutrinoTrajectoryConfigs.m_DefaultConfig);

    public static final Trajectory eightBall1 = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(2.9, -2.1, Rotation2d.fromDegrees(-60)), new Pose2d(1.6, 0, Rotation2d.fromDegrees(0))),
        NeutrinoTrajectoryConfigs.m_ReverseConfig);

    public static final Trajectory eightBall2 = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(1.6, 0, Rotation2d.fromDegrees(0)), new Pose2d(4.6, 0, Rotation2d.fromDegrees(0))),
        NeutrinoTrajectoryConfigs.m_DefaultConfig);

}
