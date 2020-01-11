/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import frc.robot.Constants.DriveConstants;

/**
 * Add your docs here.
 */
public class NeutrinoTrajectoryConfigs {

    public static final TrajectoryConfig m_DefaultConfig =
    new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                        DriveConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics)
        .addConstraint(DriveConstants.autoVoltageConstraint);

     public static final TrajectoryConfig m_ReverseConfig =
    new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                        DriveConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics)
        .addConstraint(DriveConstants.autoVoltageConstraint).setReversed(true);
}

