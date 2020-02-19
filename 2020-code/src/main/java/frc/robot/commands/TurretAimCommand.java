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
import frc.robot.util.Limelight;

public class TurretAimCommand extends CommandBase
{

    private TurretSubsystem m_turret;
    private boolean scanDirection;
    private boolean canFlipScanDirection;
    private double headingError;
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
            if (Math.abs(m_turret.getAngle()) < VisionConstants.SCAN_DIRECTION_SWITCH_RESET_THRESHOLD
                    && !canFlipScanDirection)
            {
                canFlipScanDirection = true;
            }
            if (canFlipScanDirection && Math.abs(m_turret.getAngle()) < 180)
            {
                canFlipScanDirection = false;
                scanDirection = !scanDirection;
            }
        }
        else
        {
            headingError = m_turret.getHeadingError();
            if (Math.abs(headingError) > VisionConstants.TURRET_ANGLE_TOLERANCE)
            {
                m_turret.setPower(
                    VisionConstants.TRACKING_KP * headingError - VisionConstants.TRACKING_CONSTANT_OFFSET);
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

    private void setAngle(double p_angle)
    {
        double setpoint = p_angle;
        double currentPosition = m_turret.getAngle
        double clockWise;
        double counterClockwise;

        //pick shortest rotate direction, given that it doesn't twist the cable beyond [-190, 190]
        if (setpoint > currentPosition)
        {
            counterClockwise = Math.abs(setpoint-currentPosition);
            clockWise = Math.abs((360+setpoint)- currentPosition);
            if(clockWise < counterClockwise && Math.abs(Math.abs(setpoint)-180) <= 20)
            {
                setpoint = setpoint - 360;
            }
        }
        else
        {
            clockWise = Math.abs(setpoint-currentPosition);
            counterClockwise = Math.abs(360+setpoint-currentPosition);
            if(counterClockwise < clockWise && Math.abs(Math.abs(setpoint)-180) <= 20)
            {
                setpoint = 360 + setpoint;
            }

        }

    }
}
