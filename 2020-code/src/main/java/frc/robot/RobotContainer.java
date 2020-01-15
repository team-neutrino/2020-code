/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.IntakeGetBallCommand;
import frc.robot.commands.IntakeRetractCommand;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.commands.NeutrinoRamseteCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.*;
import static edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Trajectories.ExampleTrajectory;
import frc.robot.commands.DriveDataCommand;
import frc.robot.commands.IntakeDataCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    public final DriveSubsystem m_Drive = new DriveSubsystem();
    public final IntakeSubsystem m_Intake = new IntakeSubsystem();

    public Joystick m_leftJoystick = new Joystick(0);
    public Joystick m_rightJoystick = new Joystick(1);
    XboxController m_OperatorController = new XboxController(ControllerPorts.XBOX_CONTROLLER_PORT);
    JoystickButton m_A = new JoystickButton(m_OperatorController, Button.kA.value);
    JoystickButton m_B = new JoystickButton(m_OperatorController, Button.kB.value);
    JoystickButton m_X = new JoystickButton(m_OperatorController, Button.kX.value);
    private final Trajectory m_Trajectory = ExampleTrajectory.exampleTraj;
    private final NeutrinoRamseteCommand m_autoCommand = new NeutrinoRamseteCommand(m_Drive, m_Trajectory);
    private final IntakeDataCommand m_intakeData = new IntakeDataCommand(m_Intake);


    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        final Command tankDriveCommand = new RunCommand(
            () -> m_Drive.tankDrive(m_leftJoystick.getY(), m_rightJoystick.getY()), m_Drive);
        m_Drive.setDefaultCommand(tankDriveCommand);
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() 
    {
        m_A.whenPressed(new IntakeGetBallCommand(m_Intake))
            .whenReleased(new IntakeRetractCommand(m_Intake));
        m_B.whenPressed(new IntakeDataCommand(m_Intake));
        m_X.whenPressed(new DriveDataCommand(m_Drive));
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
}
