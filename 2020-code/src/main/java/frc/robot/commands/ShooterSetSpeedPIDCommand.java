/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShooterSetSpeedPIDCommand extends PIDCommand
{
    /**
     * Creates a new ShooterSetSpeedPIDCommand.
     */
    public ShooterSetSpeedPIDCommand(ShooterSubsystem m_shooter)
    {
        super(
            // The controller that the command will use
            new PIDController(Constants.ShooterConstants.WHEEL_P, Constants.ShooterConstants.WHEEL_I,
                Constants.ShooterConstants.WHEEL_D),
            // This should return the measurement
            () -> m_shooter.getWheelEncoderDistance(),
            // This should return the setpoint (can also be a constant)
            () -> m_shooter.setpoint(),
            // This uses the output
            output -> m_shooter.setWheelMotor(output));
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
