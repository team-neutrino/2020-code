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
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Trajectories.ExampleTrajectory;
import frc.robot.subsystems.DriveSubsystem;

public class NeutrinoRamseteHandler
{
    /**
     * Creates a new NeutrinoRamseteCommand.
     */

    private DriveSubsystem m_Drive;

    public NeutrinoRamseteHandler(DriveSubsystem p_Drive)
    {
        m_Drive = p_Drive;

        System.out.println("## NeutrinoRamseteHandler constructed");
    }

    public Command getCommand( Trajectory trajectory )
    {
        // Trajectory trajectory = ExampleTrajectory.exampleTraj;
        System.out.println("## NeutrinoRamseteHandler getCommand");

        Pose2d pose = new Pose2d();
        m_Drive.resetOdometry(pose);

        PIDController leftController = new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0);
        PIDController rightController = new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0);
        RamseteController controller = new RamseteController(DriveConstants.K_RAMSETE_B, DriveConstants.K_RAMSETE_ZETA);
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.KS_VOLTS, DriveConstants.KV_VOLT_SECONDS_PER_METER, DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER);

        RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_Drive::getPose,
            controller, feedforward, DriveConstants.K_DRIVE_KINEMATICS, m_Drive::getWheelSpeeds, leftController, rightController,
            (leftVolts, rightVolts) ->
            {
                m_Drive.tankDriveVolts(leftVolts, rightVolts);

                SmartDashboard.putNumber("left actual m/s", m_Drive.getWheelSpeeds().leftMetersPerSecond);
                SmartDashboard.putNumber("left desired m/s", leftController.getSetpoint());

                SmartDashboard.putNumber("right actual m/s", m_Drive.getWheelSpeeds().rightMetersPerSecond);
                SmartDashboard.putNumber("right desired m/s", rightController.getSetpoint());
            },
            m_Drive);
        return ramseteCommand;
    }
}
