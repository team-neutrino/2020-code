/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.button.Button;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants;
import frc.robot.Constants.CanId;

public class IntakePIDSubsystem extends PIDSubsystem
{
    private TalonSRX m_IntakeFeedMotor = new TalonSRX(CanId.MOTOR_CONTROLLER_INTAKE_FEED);
    private TalonSRX m_IntakeArmMotor = new TalonSRX(CanId.MOTOR_CONTROLLER_INTAKE_POSITION);
    private DutyCycleEncoder m_DutyCycleEncoder = new DutyCycleEncoder(Constants.IntakeConstants.ENCODER_PORT);
    /**
     * Creates a new IntakePIDSubsystem.
     */
    public IntakePIDSubsystem()
    {
        super(
            // The PIDController used by the subsystem
            new PIDController(Constants.IntakeConstants.KP, Constants.IntakeConstants.KI,
                Constants.IntakeConstants.KD));
        m_DutyCycleEncoder.setDistancePerRotation(Constants.IntakeConstants.POSITION_MULTIPLIER); //degrees
    }

    @Override
    public void useOutput(double output, double setpoint)
    {
        m_IntakeArmMotor.set(ControlMode.PercentOutput, output);
        SmartDashboard.putNumber("useOutput output", output);
        SmartDashboard.putNumber("useOutput setpoint", setpoint);
    }

    @Override
    public double getMeasurement()
    {
        return m_DutyCycleEncoder.getDistance();
    }

    public void periodic()
    {
        super.periodic();
        SmartDashboard.putNumber("Arm motor current: ", m_IntakeArmMotor.getSupplyCurrent());
        SmartDashboard.putNumber("Arm motor power", m_IntakeArmMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Encoder ang.: ", m_DutyCycleEncoder.getDistance());
        SmartDashboard.putNumber("Encoder val.: ", m_DutyCycleEncoder.get());
        SmartDashboard.putBoolean("isConnected: ", m_DutyCycleEncoder.isConnected());

        if (Math.abs(getMeasurement()) > 360)
        {
            m_DutyCycleEncoder.reset();
        }
    }

    public void setAngle(double angle)
    {
        setSetpoint(angle);
        enable();
    }

    public void setIntakeOn()
    {
        m_IntakeFeedMotor.set(ControlMode.PercentOutput, Constants.IntakeConstants.INTAKE_MOTOR_POWER);
    }

    public void setIntakeOff()
    {
        m_IntakeFeedMotor.set(ControlMode.PercentOutput, 0);
    }
}