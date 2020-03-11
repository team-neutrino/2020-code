
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretSetAngleCommand extends CommandBase
{
    private double m_Angle;
    private TurretSubsystem m_Turret;
    /**
     * Creates a new Turret.
     */
    public TurretSetAngleCommand(TurretSubsystem p_Turret, double p_Angle)
    {
        // Use addRequirements() here to declare subsystem dependencies.
        m_Angle = p_Angle;
        m_Turret = p_Turret;
        addRequirements(p_Turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        m_Turret.startTimer();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        m_Turret.setAngle(m_Angle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        m_Turret.setPower(0);
        new TurretAimCommand(m_Turret).schedule();
        m_Turret.stopTimer();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        if (m_Turret.getTimer() > 15)
        {
            return true;
        }
        else
        {
            return false;
        }

    }
}
