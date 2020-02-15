/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class IntakePIDCommand extends PIDCommand
{
    /**
     * Creates a new IntakePIDCommand.
     */
    public IntakePIDCommand(IntakeSubsystem m_Intake)
    {
        super(
            // The controller that the command will use
            new PIDController(Constants.IntakeConstants.PROPORTION_COEFFICIENT,
                Constants.IntakeConstants.INTEGRAL_COEFFICIENT, Constants.IntakeConstants.DERIVATIVE_COEFFICEINT),
            // This should return the measurement
            () -> m_Intake.getEncoderValue(),
            // This should return the setpoint (can also be a constant)
            () -> m_Intake.getSetpoint(),
            // This uses the output
            output ->
            {
                m_Intake.numberPut(output);
            });
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
