/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.commands.NeutrinoRamseteCommand;
import frc.robot.commands.ShooterDirectCurrentCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.*;
import static edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.subsystems.ClimberSubsystem;
import java.nio.file.Paths;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.DriveDataCommand;
import frc.robot.commands.IntakeDataCommand;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...

    public final DriveSubsystem m_Drive = new DriveSubsystem();
    public final IntakeSubsystem m_Intake = new IntakeSubsystem();
    public final ShooterSubsystem m_Shooter = new ShooterSubsystem();
    public final LEDSubsystem m_Led = new LEDSubsystem();
    public final ClimberSubsystem m_climber = new ClimberSubsystem();

    public Joystick m_leftJoystick = new Joystick(Constants.JoystickConstants.LEFT_JOYSTICK_PORT);
    public Joystick m_rightJoystick = new Joystick(Constants.JoystickConstants.RIGHT_JOYSTICK__PORT);
    XboxController m_OperatorController = new XboxController(ControllerPorts.XBOX_CONTROLLER_PORT);
    JoystickButton m_back = new JoystickButton(m_OperatorController, Button.kBack.value);
    JoystickButton m_start = new JoystickButton(m_OperatorController, Button.kStart.value);
    JoystickButton m_A = new JoystickButton(m_OperatorController, Button.kA.value);
    JoystickButton m_B = new JoystickButton(m_OperatorController, Button.kB.value);
    JoystickButton m_X = new JoystickButton(m_OperatorController, Button.kX.value);
    private Trajectory m_Trajectory;
    private NeutrinoRamseteCommand m_autoCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        try
        {
            m_Trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/3BallAuton.wpilib.json"));
            m_autoCommand = new NeutrinoRamseteCommand(m_Drive, m_Trajectory);
        }
        catch (Exception e)
        {
        }

        final Command tankDriveCommand = new RunCommand(() -> m_Drive.tankDrive(
            joystickProcessor(m_leftJoystick.getY()), joystickProcessor(m_rightJoystick.getY())), m_Drive);
        m_Drive.setDefaultCommand(tankDriveCommand);
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        m_B.whenPressed(new IntakeDataCommand(m_Intake));
        m_X.whenPressed(new DriveDataCommand(m_Drive));
        m_A.whenHeld(new ShooterDirectCurrentCommand(m_Shooter));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return m_autoCommand;
    }

    /**
     * Applies deadzoning and curve to the joystick input
     *
     * @return A processed joystick input
     */
    private double joystickProcessor(double input)
    {
        if (Math.abs(input) > Constants.JoystickConstants.DEADZONE_SIZE)
        {
            double absoluteValue = Math.abs(input);
            double deadzoneCorrectedAbsoluteValue = (1 / (1 - Constants.JoystickConstants.DEADZONE_SIZE))
                    * (absoluteValue - 1.0) + 1.0;
            return Math.pow(deadzoneCorrectedAbsoluteValue, Constants.JoystickConstants.JOYSTICK_CURVE)
                    * (absoluteValue / input);
        }
        else
        {
            return 0.0;
        }
    }
}
