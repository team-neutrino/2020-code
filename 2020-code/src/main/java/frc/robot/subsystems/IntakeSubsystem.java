/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanId;
import frc.robot.Constants.IntakeConstants;

/**
 * Add your docs here.
 */

public class IntakeSubsystem extends SubsystemBase
{
    private AnalogPotentiometer m_adjustMotorPotentiometer = new AnalogPotentiometer(
        Constants.IntakeConstants.ADJUST_MOTOR_ENCODER);
    private TalonSRX m_intakeMotor = new TalonSRX(CanId.MOTOR_CONTROLLER_INTAKE);
    private TalonSRX m_intakeAdjustMotor = new TalonSRX(CanId.MOTOR_CONTROLLER_INTAKE_ADJUST);
    private PowerDistributionPanel PDP = new PowerDistributionPanel();

    public void setIntakeOn()
    {
        m_intakeMotor.set(ControlMode.PercentOutput, Constants.IntakeConstants.INTAKE_MOTOR_POWER);
    }

    public void setIntakeOff()
    {
        m_intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    public double getSetpoint()
    {
        return Constants.IntakeConstants.ARM_DOWN_ANGLE;
    }

    public double getPotentiometerReading()
    {
        return m_adjustMotorPotentiometer.get();
    }

    public void setPositionMotorDown(double PIDPower)
    {
        m_intakeAdjustMotor.set(ControlMode.PercentOutput, PIDPower);
    }

    public void printPDPCurrent()
    {
        double currentIntakeMotor = m_intakeMotor.getSupplyCurrent();
        SmartDashboard.putNumber("MOTOR_CONTROLLER_INTAKE: ", currentIntakeMotor);
    }

    public void intakePrinter()
    {
        //SmartDashboard.putNumber("Arm setpoint: ", PID.getSetpoint());
        SmartDashboard.putNumber("Potentiometer reading: ", getPotentiometerReading());
    }

}