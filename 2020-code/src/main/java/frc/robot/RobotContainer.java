/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.*;
import static edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.subsystems.ClimberSubsystem;
import java.nio.file.Paths;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    public final IntakePIDSubsystem m_Intake = new IntakePIDSubsystem();
    public final ShooterSubsystem m_Shooter = new ShooterSubsystem();
    public final DriveSubsystem m_Drive = new DriveSubsystem();
    public final LEDSubsystem m_Led = new LEDSubsystem();
    public final ClimberSubsystem m_climber = new ClimberSubsystem();
    public final HopperSubsystem m_Hopper = new HopperSubsystem();
    public final TurretSubsystem m_Turret = new TurretSubsystem();

    private Joystick m_leftJoystick = new Joystick(Constants.JoystickConstants.LEFT_JOYSTICK_PORT);
    private Joystick m_rightJoystick = new Joystick(Constants.JoystickConstants.RIGHT_JOYSTICK__PORT);
    XboxController m_OperatorController = new XboxController(ControllerPorts.XBOX_CONTROLLER_PORT);
    JoystickButton m_back = new JoystickButton(m_OperatorController, Button.kBack.value);
    JoystickButton m_start = new JoystickButton(m_OperatorController, Button.kStart.value);
    JoystickButton m_A = new JoystickButton(m_OperatorController, Button.kA.value);
    JoystickButton m_B = new JoystickButton(m_OperatorController, Button.kB.value);
    JoystickButton m_X = new JoystickButton(m_OperatorController, Button.kX.value);
    JoystickButton m_rightJoystickButton = new JoystickButton(m_OperatorController, Button.kStickRight.value);
    JoystickButton m_Y = new JoystickButton(m_OperatorController, Button.kY.value);
    JoystickButton m_BumperLeft = new JoystickButton(m_OperatorController, Button.kBumperLeft.value);
    TriggerToBoolean m_TriggerLeft = new TriggerToBoolean(m_OperatorController, Axis.kLeftTrigger.value,
        Constants.IntakeConstants.LEFT_TRIGGER_THRESHOLD);

    private Trajectory m_Trajectory;
    private Trajectory auton_Trajectory;
    private NeutrinoRamseteCommand m_autoCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {

        try
        {
            m_Trajectory = TrajectoryUtil.fromPathweaverJson(
                Paths.get("/home/lvuser/deploy/output/DriveStraightTest.wpilib.json"));
            Pose2d bOrigin = m_Drive.getPose();
            auton_Trajectory = m_Trajectory.relativeTo(bOrigin);
            m_autoCommand = new NeutrinoRamseteCommand(m_Drive, m_Trajectory);
        }
        catch (Exception e)
        {
            e.printStackTrace();
            System.out.println("This didnt work" + e);
        }
        final Command tankDriveCommand = new RunCommand(
            () -> m_Drive.tankDrive(m_leftJoystick.getY(), m_rightJoystick.getY()), m_Drive);
        m_Drive.setDefaultCommand(tankDriveCommand);
        m_Hopper.setDefaultCommand(new HopperDefaultCommand(m_Hopper));
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        m_X.whenPressed(new DriveDataCommand(m_Drive));
        m_A.whenHeld(new ShooterSetSpeedCommand(m_Shooter));
        m_BumperLeft.whileHeld(new InstantCommand(m_Hopper::intake, m_Hopper), false);
        m_rightJoystickButton.toggleWhenActive(
            new TurretOverrideCommand(m_Turret, () -> m_OperatorController.getX(Hand.kRight)));
        m_TriggerLeft.whenActive(new InstantCommand(m_Intake::setIntakeOn, m_Intake).alongWith(
            new InstantCommand(() -> m_Intake.setAngle(Constants.IntakeConstants.ARM_DOWN_ANGLE))));
        m_TriggerLeft.whenInactive(new InstantCommand(m_Intake::setIntakeOff, m_Intake).alongWith(
            new InstantCommand(() -> m_Intake.setAngle(Constants.IntakeConstants.ARM_UP_ANGLE))));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {

        RamseteCommand ramseteCommand = new RamseteCommand(auton_Trajectory, m_Drive::getPose,
            new RamseteController(DriveConstants.K_RAMSETE_B, DriveConstants.K_RAMSETE_ZETA),
            new SimpleMotorFeedforward(DriveConstants.KS_VOLTS, DriveConstants.KV_VOLT_SECONDS_PER_METER,
                DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
            DriveConstants.K_DRIVE_KINEMATICS, m_Drive::getWheelSpeeds,
            new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0), new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0),
            // RamseteCommand passes volts to the callback
            m_Drive::tankDriveVolts, m_Drive);

        //TODO: transform coordinates

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_Drive.tankDriveVolts(0, 0));
    }

}
