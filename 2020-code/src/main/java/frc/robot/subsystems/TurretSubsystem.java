/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.TurretConstants;
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
        m_turretMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        m_turretMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }

    public void setAngle(double p_angle)
    {
        //TODO: Ensure the talon position thing sets to degrees
        m_turretMotor.set(ControlMode.Position, p_angle);
    }

    public double getTurretAngle()
    {
        //TODO: make sure this actually returns the correct angle
        return m_turretMotor.getSelectedSensorPosition() * TurretConstants.ANGLE_SCALE;
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
