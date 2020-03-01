/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.TurretSubsystem;

public class TurretAimCommand extends CommandBase
{

    private TurretSubsystem m_turret;
    private boolean scanDirection;
    private boolean canFlipScanDirection;
    private double m_headingError;
    private double currentPosition;

    /**
     * Creates a new TurretAimCommand.
     */
    public TurretAimCommand(TurretSubsystem p_turret)
    {
        addRequirements(p_turret);
        m_turret = p_turret;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        scanDirection = false;
        canFlipScanDirection = false;
        m_headingError = 0.0;
        m_turret.setLightOn();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        // If there is no valid target, sets power to 0
        if (m_turret.getValidTarget() == 0)
        {
            m_turret.setPower(0);
        }
        else
        {
            // Sets angle to desired turret angle plus error if there is a target
            m_headingError = m_turret.getHeadingError();
            currentPosition = m_turret.getTurretAngle();
            SmartDashboard.putNumber("Turretangle", currentPosition);
            m_turret.autoSetAngle(turretLimit(currentPosition + m_headingError));
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        // Stops turret when command ends
        m_turret.setPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }

    /**
     * @return Shortest distance setpoint to turn to that wont break wires
     **/
    private double turretLimit(double p_angle)
    {
        double setpoint = p_angle;
        double forwardRotationLimit = 135;
        double backwardRotationLimit = -135;

        if (setpoint > forwardRotationLimit)
        {
            setpoint = forwardRotationLimit;
        }
        if (setpoint < backwardRotationLimit)
        {
            setpoint = backwardRotationLimit;
        }
        return setpoint;
    }
}
