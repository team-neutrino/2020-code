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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;

import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase
{
    private TalonSRX m_turretMotor = new TalonSRX(CanId.MOTOR_CONTROLLER_TURRET);
    private NetworkTableEntry tX;
    private NetworkTableEntry tV;
    private NetworkTableEntry ledMode;
    private double m_turretAngle;
    private double m_headingError;
    private double m_getValidTarget;
    /**
     * Creates a new TurretSubsystem.
     */
    public TurretSubsystem()
    {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        tX = table.getEntry("tx");
        tV = table.getEntry("tv");
        ledMode = table.getEntry("ledMode");
        m_turretMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        m_turretMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Turret Angle", getTurretAngle());
        m_turretAngle = m_turretMotor.getSelectedSensorPosition();
        m_headingError = tX.getDouble(0.0);
        m_getValidTarget = tV.getDouble(0.0);
    }

    public void autoSetAngle(double p_angle)
    {
        double currentAngle = getTurretAngle();
        double kP = 0.07;
        double setpoint = p_angle;
        double error = setpoint - currentAngle;
        m_turretMotor.set(ControlMode.PercentOutput, kP * error);
    }

    public void setPointSetAngle(double p_angle)
    {
        double currentAngle = getTurretAngle();
        double kP = 0.12;
        double setpoint = p_angle;
        double error = setpoint - currentAngle;
        m_turretMotor.set(ControlMode.PercentOutput, kP * error);
    }

    public double getTurretAngle()
    {

        return m_turretAngle;
    }

    public void setPower(double power)
    {
        m_turretMotor.set(ControlMode.PercentOutput, power);
    }

    /**
     * @return returns horizontal heading error in degrees
     */
    public double getHeadingError()
    {
        return m_headingError;
    }

    /**
     * @return 1 if valid target, 0 if no valid target
     */
    public double getValidTarget()
    {
        return m_getValidTarget;
    }

    public void toggleLight()
    {
        Number mode = ledMode.getNumber(0);
        if (mode.intValue() == 0 || mode.intValue() == 3)
        {
            ledMode.setNumber(1);

        }
        else if (mode.intValue() == 1)
        {
            ledMode.setNumber(3);
        }
    }

    public int getLightValue()
    {
        return ledMode.getNumber(0).intValue();
    }

    public void setLightOn()
    {
        ledMode.setNumber(3);
    }

    public void setLightOff()
    {
        ledMode.setNumber(1);
    }
}
