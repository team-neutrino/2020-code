/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretOverrideCommand extends CommandBase
{
    private final TurretSubsystem m_turret;
    private final DoubleSupplier m_overridePower;
    /**
     * Creates a new TurretOverrideCommand.
     */
    public TurretOverrideCommand(TurretSubsystem p_turret, DoubleSupplier p_overridePower)
    {
        addRequirements(p_turret);
        m_turret = p_turret;
        m_overridePower = p_overridePower;
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
        m_turret.setPower(m_overridePower.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        m_turret.setPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
