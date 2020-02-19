/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
            m_turret.setPower(Constants.VisionConstants.SCAN_SPEED * (scanDirection ? 1.0 : -1.0));
            if (Math.abs(m_turret.getAngle()) < Constants.VisionConstants.SCAN_DIRECTION_SWITCH_RESET_THRESHOLD
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
            if (headingError > Constants.VisionConstants.LOCKON_ANGLE_THRESHOLD)
            {
                m_turret.setPower(Constants.VisionConstants.TRACKING_KP * headingError
                        - Constants.VisionConstants.TRACKING_CONSTANT_OFFSET);
            }
            else if (headingError < -Constants.VisionConstants.LOCKON_ANGLE_THRESHOLD)
            {
                m_turret.setPower(Constants.VisionConstants.TRACKING_KP * headingError
                        + Constants.VisionConstants.TRACKING_CONSTANT_OFFSET);
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
}
