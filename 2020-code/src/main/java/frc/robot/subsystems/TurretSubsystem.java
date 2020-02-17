/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;
import frc.robot.util.Limelight;

public class TurretSubsystem extends SubsystemBase
{
    private TalonSRX m_turretMotor = new TalonSRX(CanId.MOTOR_CONTROLLER_TURRET);
    private Limelight m_limelight = new Limelight();
    /**
     * Creates a new TurretSubsystem.
     */
    public TurretSubsystem()
    {

    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }

    public void setAngle()
    {

    }

    public void getAngle()
    {

    }

    public void setPower(double power)
    {
        m_turretMotor.set(ControlMode.PercentOutput, power);
    }

    public double getHeadingError()
    {
        return m_limelight.getXAngle();
    }

    public double getValidTarget()
    {
        return m_limelight.hasTarget();
    }
}
