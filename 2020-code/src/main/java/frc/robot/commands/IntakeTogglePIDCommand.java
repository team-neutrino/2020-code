/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeTogglePIDCommand extends CommandBase
{
    IntakeSubsystem m_Intake;
    /**
     * Creates a new IntakeTogglePIDCommand.
     */
    public IntakeTogglePIDCommand(IntakeSubsystem p_Intake)
    {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(p_Intake);
        m_Intake = p_Intake;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        new PIDCommand(
            new PIDController(Constants.IntakeConstants.PROPORTION_COEFFICIENT,
                Constants.IntakeConstants.INTEGRAL_COEFFICIENT, Constants.IntakeConstants.DERIVATIVE_COEFFICEINT),
            // This should return the measurement
            () -> m_Intake.getEncoderValue(),
            // This should return the setpoint (can also be a constant)
            () -> m_Intake.getSetpointDown(),
            // This uses the output
            output ->
            {
                m_Intake.numberPut(output);
            });
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        new PIDCommand(
            new PIDController(Constants.IntakeConstants.PROPORTION_COEFFICIENT,
                Constants.IntakeConstants.INTEGRAL_COEFFICIENT, Constants.IntakeConstants.DERIVATIVE_COEFFICEINT),
            // This should return the measurement
            () -> m_Intake.getEncoderValue(),
            // This should return the setpoint (can also be a constant)
            () -> m_Intake.getSetpointUp(),
            // This uses the output
            output ->
            {
                m_Intake.numberPut(output);
            });
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
