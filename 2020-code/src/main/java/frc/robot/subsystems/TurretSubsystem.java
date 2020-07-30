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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanId;

import edu.wpi.first.wpilibj.Timer;

public class TurretSubsystem extends SubsystemBase
{
    private TalonSRX m_turretMotor = new TalonSRX(CanId.MOTOR_CONTROLLER_TURRET);
    private Timer m_Timer1 = new Timer();

    private NetworkTableEntry tX;
    private NetworkTableEntry tV;
    private NetworkTableEntry ledMode;
    private NetworkTableEntry camMode;
    private double m_turretAngle;
    private double m_headingError;
    private double m_getValidTarget;
    private double m_dynamicOffset;
    private double currentPosition;

    /**
     * Creates a new TurretSubsystem.
     */
    public TurretSubsystem()
    {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        tX = table.getEntry("tx");
        tV = table.getEntry("tv");
        ledMode = table.getEntry("ledMode");
        camMode = table.getEntry("camMode");
        m_turretMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        m_turretMotor.setNeutralMode(NeutralMode.Brake);
        m_dynamicOffset = m_turretMotor.getSelectedSensorPosition();

    }

    @Override
    public void periodic()
    {
        m_turretAngle = m_turretMotor.getSelectedSensorPosition() - m_dynamicOffset;
        m_headingError = tX.getDouble(0.0);
        m_getValidTarget = tV.getDouble(0.0);
    }

    public void startTimer()
    {
        m_Timer1.start();
    }

    public double getTimer()
    {
        return m_Timer1.get();
    }

    public void stopTimer()
    {
        m_Timer1.stop();
    }

    /**
     * Sets angle in autonomous mode, then begins tracking goal after 0.5 seconds.
     *
     * @param angle Angle set relative to robot
     */
    public void setAngle(double angle)
    {
        if (m_Timer1.get() < 0.5)
        {
            setpointSetAngle(angle);
        }
        else
        {
            if (currentPosition < 90)
            {
                setpointSetAngle(turretLimit(getTurretAngle() + getHeadingError()));
            }
            else
            {
                setPower(0);
            }
        }
    }

    public void autoSetAngle()
    {
        if (getValidTarget() == 0)
        {
            setPower(0);
        }
        else
        {
            // Sets angle to desired turret angle plus error if there is a target
            setpointSetAngle(turretLimit(getTurretAngle() + getHeadingError()));
        }
    }

    public void setpointSetAngle(double p_angle)
    {
        double currentAngle = getTurretAngle();
        double setpoint = p_angle;
        double error = setpoint - currentAngle;
        m_turretMotor.set(ControlMode.PercentOutput, Constants.TurretConstants.kP * error);
    }

    public double getTurretAngle()
    {
        return m_turretAngle;
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

    public void setDriverCamMode()
    {
        camMode.setNumber(1);
    }

    public void setVisionCamMode()
    {
        camMode.setNumber(0);
    }

    public void setPower(double power)
    {
        m_turretMotor.set(ControlMode.PercentOutput, power);
    }

    /**
     * @return Shortest distance setpoint to turn to that wont break wires
     **/
    public double turretLimit(double p_angle)
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

    public void setTurretMotorOff() 
    {
        m_turretMotor.set(ControlMode.PercentOutput, 0);
    }
}
