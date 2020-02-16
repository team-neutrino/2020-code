/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;

public class HopperDefaultCommand extends CommandBase
{
    /**
     * Creates a new HopperDefaultCommand.
     */

    private HopperSubsystem m_HopperSubsystem;
    boolean bottomBeam;
    boolean topBeam;
    public HopperDefaultCommand(HopperSubsystem p_Hopper)
    {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(p_Hopper);
        m_HopperSubsystem = p_Hopper;
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
        bottomBeam = m_HopperSubsystem.bottomBeamStatus();
        topBeam = m_HopperSubsystem.topBeamStatus();
        SmartDashboard.putBoolean("Beam Break 1", bottomBeam);
        SmartDashboard.putBoolean("Beam Break 2", topBeam);
        if (topBeam == false || m_HopperSubsystem.getTime() >= 0.1)
        {
            m_HopperSubsystem.stop();
            return;
        }
        if (bottomBeam == false)
        {
            m_HopperSubsystem.intake();
            m_HopperSubsystem.stopTimer();
        }
        else if (m_HopperSubsystem.getPrevBotBeam() == false && bottomBeam == true)
        {
            m_HopperSubsystem.resetTimer();
            m_HopperSubsystem.startTimer();
        }
        m_HopperSubsystem.setPrevBotBeam(bottomBeam);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
