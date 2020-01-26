/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class NeutrinoRamseteCommand extends RamseteCommand
{
    /**
     * Creates a new NeutrinoRamseteCommand.
     */

    private DriveSubsystem m_Drive;

    public NeutrinoRamseteCommand(DriveSubsystem p_Drive, Trajectory p_Trajectory)
    {

        super(p_Trajectory, p_Drive::getPose,
            new RamseteController(DriveConstants.K_RAMSETE_B, DriveConstants.K_RAMSETE_ZETA),
            new SimpleMotorFeedforward(DriveConstants.KS_VOLTS, DriveConstants.KV_VOLT_SECONDS_PER_METER,
                DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
            DriveConstants.K_DRIVE_KINEMATICS, p_Drive::getWheelSpeeds,
            new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0), new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0),
            // RamseteCommand passes volts to the callback
            p_Drive::tankDriveVolts, p_Drive);
        m_Drive = p_Drive;

    }

    @Override
    public void end(boolean interrupted)
    {
        super.end(interrupted);
        m_Drive.tankDriveVolts(0, 0);
    }
}
