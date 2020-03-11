/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class CamModeToggleCommand extends CommandBase
{
    /**
     * Creates a new CamModeToggleCommand. When initalized, sets limelight to driver mode. When cancelled, sets limeight
     * to vision processing mode.
     */
    private TurretSubsystem m_Turret;
    public CamModeToggleCommand(TurretSubsystem p_Turret)
    {
        m_Turret = p_Turret;
        addRequirements(p_Turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        m_Turret.setDriverCamMode();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        m_Turret.setVisionCamMode();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
