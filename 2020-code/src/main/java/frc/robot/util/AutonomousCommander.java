
package frc.robot.util;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Trajectories.ExampleTrajectory;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousCommander
{
    private DriveSubsystem m_Drive;

    public AutonomousCommander(DriveSubsystem p_Drive)
    {
        m_Drive = p_Drive;
        System.out.println("## AutonomousCommander constructed");
    }

    public Command getCommand()
    {
        System.out.println("## AutonomousCommander getCommand");

        // TODO: maek a good trajectory or even a shuffleboard selector
        Trajectory trajectory = ExampleTrajectory.exampleTraj;
        m_Drive.resetOdometry(new Pose2d());

        // objects needed by the RamseteCommand
        PIDController leftController = new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0);
        PIDController rightController = new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0);
        RamseteController controller = new RamseteController(DriveConstants.K_RAMSETE_B, DriveConstants.K_RAMSETE_ZETA);
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.KS_VOLTS,
            DriveConstants.KV_VOLT_SECONDS_PER_METER, DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER);

        RamseteCommand sixBallTraj0 = new RamseteCommand(trajectory, m_Drive::getPose, controller, feedforward,
            DriveConstants.K_DRIVE_KINEMATICS, m_Drive::getWheelSpeeds, leftController, rightController,
            m_Drive::tankDriveVolts, m_Drive);

        return sixBallTraj0.andThen(() -> m_Drive.tankDriveVolts(0, 0));
    }
}
