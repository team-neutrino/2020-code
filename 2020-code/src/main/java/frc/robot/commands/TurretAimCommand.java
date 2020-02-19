/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.TurretSubsystem;

public class TurretAimCommand extends CommandBase
{

    private TurretSubsystem m_turret;
    private boolean scanDirection;
    private boolean canFlipScanDirection;
    private double headingError;
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
        headingError = 0.0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        if (m_turret.getValidTarget() == 0)
        {
            m_turret.setPower(VisionConstants.SCAN_SPEED * (scanDirection ? 1.0 : -1.0));
            if (Math.abs(m_turret.getTurretAngle()) < VisionConstants.SCAN_DIRECTION_SWITCH_RESET_THRESHOLD
                    && !canFlipScanDirection)
            {
                canFlipScanDirection = true;
            }
            if (canFlipScanDirection && Math.abs(m_turret.getTurretAngle()) < 180)
            {
                canFlipScanDirection = false;
                scanDirection = !scanDirection;
            }
        }
        else
        {
            headingError = m_turret.getHeadingError();
            currentPosition = m_turret.getTurretAngle();
            if (Math.abs(headingError) > VisionConstants.TURRET_ANGLE_TOLERANCE)
            {
                m_turret.setAngle(turretLimit(headingError + currentPosition));
            }
            else
            {
                //fire stuff
            }
        }
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

    /**
     * Takes an angle setopint relative to robot and returns shortest distance setpoint to turn to 
     * that wont break wires. Credit to team 3476 Code Orange for this logic.
    **/
    private double turretLimit(double p_angle)
    {
        double setpoint = p_angle;
        double clockWise;
        double counterClockwise;

        //normalize requested angle on [-180,180]
        setpoint -= 360.0*Math.round(setpoint/360.0);

        //pick shortest rotate direction, given that it doesn't twist the cable beyond [-190, 190]
        if (setpoint > currentPosition)
        {
            counterClockwise = Math.abs(setpoint - currentPosition);
            clockWise = Math.abs((360 + setpoint) - currentPosition);
            if (clockWise < counterClockwise && Math.abs(Math.abs(setpoint) - 180) <= 20)
            {
                setpoint = setpoint - 360;
                return setpoint;
            }
        }
        else
        {
            clockWise = Math.abs(setpoint - currentPosition);
            counterClockwise = Math.abs(360 + setpoint - currentPosition);
            if (counterClockwise < clockWise && Math.abs(Math.abs(setpoint) - 180) <= 20)
            {
                setpoint = 360 + setpoint;
                return setpoint;
            }
        }
        return setpoint;
    }
}
