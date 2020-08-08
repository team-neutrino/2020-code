/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.*;
import static edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.subsystems.*;
import frc.robot.util.TriggerToBoolean;
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
    private final IntakePIDSubsystem m_Intake = new IntakePIDSubsystem();
    private final ShooterSubsystem m_Shooter = new ShooterSubsystem();
    private final DriveSubsystem m_Drive = new DriveSubsystem();
    private final ClimberSubsystem m_climber = new ClimberSubsystem();
    private final HopperSubsystem m_Hopper = new HopperSubsystem(m_Shooter);
    private final TurretSubsystem m_Turret = new TurretSubsystem();
    private final DriverViewSubsystem m_DriverView = new DriverViewSubsystem(m_Shooter, m_Turret, m_Hopper);

    private Joystick m_leftJoystick = new Joystick(Constants.JoystickConstants.LEFT_JOYSTICK_PORT);
    private Joystick m_rightJoystick = new Joystick(Constants.JoystickConstants.RIGHT_JOYSTICK__PORT);
    private XboxController m_OperatorController = new XboxController(ControllerPorts.XBOX_CONTROLLER_PORT);
    private JoystickButton m_back = new JoystickButton(m_OperatorController, Button.kBack.value);
    private JoystickButton m_start = new JoystickButton(m_OperatorController, Button.kStart.value);
    private JoystickButton m_A = new JoystickButton(m_OperatorController, Button.kA.value);
    private JoystickButton m_B = new JoystickButton(m_OperatorController, Button.kB.value);
    private JoystickButton m_X = new JoystickButton(m_OperatorController, Button.kX.value);
    private JoystickButton m_rightJoystickButton = new JoystickButton(m_OperatorController, Button.kStickRight.value);
    private JoystickButton m_Y = new JoystickButton(m_OperatorController, Button.kY.value);
    private JoystickButton m_BumperLeft = new JoystickButton(m_OperatorController, Button.kBumperLeft.value);
    private JoystickButton m_BumperRight = new JoystickButton(m_OperatorController, Button.kBumperRight.value);
    private JoystickButton m_LJoy8 = new JoystickButton(m_leftJoystick, 8);
    private TriggerToBoolean m_TriggerLeft = new TriggerToBoolean(m_OperatorController, Axis.kLeftTrigger.value,
        Constants.IntakeConstants.LEFT_TRIGGER_THRESHOLD);
    private TriggerToBoolean m_TriggerRight = new TriggerToBoolean(m_OperatorController, Axis.kRightTrigger.value,
        Constants.IntakeConstants.RIGHT_TRIGGER_THRESHOLD);
    private POVButton m_UpPovButton = new POVButton(m_OperatorController, 0);
    private POVButton m_RightPovButton = new POVButton(m_OperatorController, 90);
    private POVButton m_DownPovButton = new POVButton(m_OperatorController, 180);
    private SixBallAuto m_SixBallAuto;
    private ThreeAuton m_ThreeAuton;
    private DumpAuton m_DumpAuton;
    private EightBallAuto m_EightBallAuto;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        m_Hopper.setDefaultCommand(new HopperDefaultCommand(m_Hopper));
        //m_Turret.setDefaultCommand(new TurretAimCommand(m_Turret));
        m_SixBallAuto = new SixBallAuto(m_Shooter, m_Hopper, m_Intake, m_Drive, m_Turret);
        m_DumpAuton = new DumpAuton(m_Shooter, m_Hopper, m_Intake, m_Drive, m_Turret);
        m_ThreeAuton = new ThreeAuton(m_Shooter, m_Hopper, m_Drive, 10);
        m_EightBallAuto = new EightBallAuto(m_Shooter, m_Hopper, m_Intake, m_Drive, m_Turret);
        //limelightFeed = new HttpCamera("limeight", "http://limelight.local:5800/stream.mjpg");
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        m_start.whileHeld(new InstantCommand(m_climber::elevatorUp, m_climber), true).whenReleased(
            m_climber::elevatorStop, m_climber);

        m_X.whileHeld(new InstantCommand(m_climber::elevatorDown, m_climber), true).whenReleased(
            m_climber::elevatorStop, m_climber);

        m_back.whileHeld(new ParallelCommandGroup(new InstantCommand(m_climber::winchClimb, m_climber))).whenReleased(
            new InstantCommand(m_climber::winchStop, m_climber));

        m_LJoy8.whenHeld(new InstantCommand(m_climber::winchReverse, m_climber)).whenReleased(m_climber::winchStop,
            m_climber);

        m_A.whenHeld( new ShooterSetSpeedCommand(m_Shooter, 80000));
        m_Y.whenHeld( new ShooterSetSpeedCommand(m_Shooter, 95000));
        
        m_BumperLeft.whileHeld(new InstantCommand(m_Hopper::towerShoot, m_Hopper), false).whenReleased(
            (new InstantCommand(m_Hopper::stop, m_Hopper)));
        m_BumperRight.whileHeld(new InstantCommand(m_Hopper::reverse, m_Hopper), false).whenReleased(
            (new InstantCommand(m_Hopper::stop, m_Hopper)));
        m_rightJoystickButton.toggleWhenActive(
            new TurretOverrideCommand(m_Turret, () -> m_OperatorController.getX(Hand.kRight)));

        m_TriggerLeft.whenActive(
            new InstantCommand(m_Intake::setIntakeOn, m_Intake).alongWith(new InstantCommand(m_Intake::setArmDown)));
        m_TriggerLeft.whenInactive(new InstantCommand(m_Intake::setIntakeOff, m_Intake).alongWith(
            new InstantCommand(() -> m_Intake.setAngle(Constants.IntakeConstants.ARM_UP_ANGLE))));

        m_UpPovButton.whileHeld(new InstantCommand(() -> m_Turret.setpointSetAngle(-90), m_Turret)).whenReleased(
            new InstantCommand(() -> m_Turret.setPower(0), m_Turret));
        m_RightPovButton.whileHeld(new InstantCommand(() -> m_Turret.setpointSetAngle(0), m_Turret)).whenReleased(
            new InstantCommand(() -> m_Turret.setPower(0), m_Turret));
        m_DownPovButton.whileHeld(new InstantCommand(() -> m_Turret.setpointSetAngle(90), m_Turret)).whenReleased(
            new InstantCommand(() -> m_Turret.setPower(0), m_Turret));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        m_Drive.initAuton();
        // return m_SixBallAuto;
        // return m_ThreeAuton;
        //return m_DumpAuton;
        return m_EightBallAuto;
    }

    public void teleopInit()
    {
        configureButtonBindings();
        final Command tankDriveCommand = new RunCommand(
            () -> m_Drive.tankDrive(m_leftJoystick.getY(), m_rightJoystick.getY()), m_Drive);
        m_Drive.setDefaultCommand(tankDriveCommand);
    }

}
