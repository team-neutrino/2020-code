/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAuton extends CommandBase
{
    private ShooterSubsystem m_Shooter;
    private HopperSubsystem m_Hopper;
    private double m_Duration;
    private Timer m_Timer = new Timer();
    private double m_Velocity;

    /**
     * Creates a new ShootAuton.
     */
    public ShootAuton(ShooterSubsystem p_Shooter, HopperSubsystem p_Hopper, double p_Duration, double p_Velocity)
    {
        addRequirements(p_Shooter, p_Hopper);
        m_Shooter = p_Shooter;
        m_Hopper = p_Hopper;
        m_Duration = p_Duration;
        m_Velocity = p_Velocity;
        //TODO: add a velocity parameter
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        m_Timer.start();
        m_Shooter.setVelocity(m_Velocity);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        if (m_Timer.get() > 1.2)
        {
            m_Hopper.towerShoot();
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        m_Shooter.setPower(0);
        m_Hopper.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return m_Timer.get() >= m_Duration;
    }
}
