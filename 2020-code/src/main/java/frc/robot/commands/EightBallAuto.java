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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Trajectories.ExampleTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakePIDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class EightBallAuto extends SequentialCommandGroup
{
    /**
     * Creates a new EightBallAuton.
     */
    public EightBallAuto(ShooterSubsystem p_Shooter, HopperSubsystem p_Hopper, IntakePIDSubsystem p_Intake,
            DriveSubsystem p_Drive, TurretSubsystem p_Turret)
    {
        Trajectory trajectory0 = ExampleTrajectory.eightBall0;
        Trajectory trajectory1 = ExampleTrajectory.eightBall1;
        Trajectory trajectory2 = ExampleTrajectory.eightBall2;

        PIDController leftController = new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0);
        PIDController rightController = new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0);

        RamseteController controller = new RamseteController(DriveConstants.K_RAMSETE_B, DriveConstants.K_RAMSETE_ZETA);

        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.KS_VOLTS,
            DriveConstants.KV_VOLT_SECONDS_PER_METER, DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER);

        // Six ball auton trajectory
        RamseteCommand eightBallTraj0 = new RamseteCommand(trajectory0, p_Drive::getPose, controller, feedforward,
            DriveConstants.K_DRIVE_KINEMATICS, p_Drive::getWheelSpeeds, leftController, rightController,
            p_Drive::tankDriveVolts, p_Drive);
        RamseteCommand eightBallTraj1 = new RamseteCommand(trajectory1, p_Drive::getPose, controller, feedforward,
            DriveConstants.K_DRIVE_KINEMATICS, p_Drive::getWheelSpeeds, leftController, rightController,
            p_Drive::tankDriveVolts, p_Drive);
        RamseteCommand eightBallTraj2 = new RamseteCommand(trajectory2, p_Drive::getPose, controller, feedforward,
            DriveConstants.K_DRIVE_KINEMATICS, p_Drive::getWheelSpeeds, leftController, rightController,
            p_Drive::tankDriveVolts, p_Drive);

        addCommands(
            // TurretSetAngleCommand coexists with the default TurretAimCommand
            // as a ParallelCommandGroup
            new InstantCommand(() -> p_Turret.setAngle(45)).alongWith(new SequentialCommandGroup(
                new InstantCommand(p_Intake::setArmDown), new InstantCommand(p_Intake::setIntakeOn, p_Intake),
                new WaitCommand(.25), eightBallTraj0, new InstantCommand(() -> p_Drive.tankDrive(0, 0), p_Drive),
                new ShootAuton(p_Shooter, p_Hopper, 3, 80000), eightBallTraj1,
                new InstantCommand(() -> p_Drive.tankDrive(0, 0), p_Drive), eightBallTraj2,
                new InstantCommand(() -> p_Drive.tankDrive(0, 0), p_Drive),
                new ShootAuton(p_Shooter, p_Hopper, 7, 85000))));
    }
}
