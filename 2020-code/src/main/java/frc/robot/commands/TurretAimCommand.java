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
    private Timer m_Timer = new Timer(); /* add */
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
        if (m_turret.getValidTarget() == 0)
        {
            // m_turret.setPower(VisionConstants.SCAN_SPEED * (scanDirection ? 1.0 : -1.0));
            // if (Math.abs(m_turret.getTurretAngle()) < VisionConstants.SCAN_DIRECTION_SWITCH_RESET_THRESHOLD
            //         && !canFlipScanDirection)
            // {
            //     canFlipScanDirection = true;
            // }
            // if (canFlipScanDirection && Math.abs(m_turret.getTurretAngle()) < 180)
            // {
            //     canFlipScanDirection = false;
            //     scanDirection = !scanDirection;
            // }
            m_turret.setPower(0);
        }
        else
        {

            m_headingError = m_turret.getHeadingError();
            currentPosition = m_turret.getTurretAngle();
            SmartDashboard.putNumber("Turretangle", currentPosition);
            double angleSet = currentPosition + m_headingError;
            m_turret.autoSetAngle(turretLimit(currentPosition + m_headingError));
        }

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
        /* add */
        /*
         * if (m_Timer.get() > 0.3) { return true; } else { return false; }
         */
        /* add */
        return false;
    }

    /**
     * Takes an angle setopint relative to robot and returns shortest distance setpoint to turn to that wont break
     * wires.
     **/
    private double turretLimit(double p_angle)
    {
        double setpoint = p_angle;
        // double rotationLimit = 180;
        // double rotationOverlap = 20;
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
