/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.CanId;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/**
 * Add your docs here.
 */

public class IntakeSubsystem extends SubsystemBase
{
    private TalonSRX m_intakeMotor = new TalonSRX(CanId.MOTOR_CONTROLLER_INTAKE);
    private TalonSRX m_intakeAdjustMotor = new TalonSRX(CanId.MOTOR_CONTROLLER_INTAKE_ADJUST);

    public IntakeSubsystem()
    {
        m_intakeAdjustMotor.configFactoryDefault();
        m_intakeAdjustMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, Constants.PIDConstants.PID_ID,
            Constants.PIDConstants.TIMEOUT_MS);
        m_intakeAdjustMotor.setSensorPhase(Constants.IntakeConstants.SENSOR_PHASE);
        m_intakeAdjustMotor.config_kP(Constants.PIDConstants.PID_ID, Constants.PIDConstants.PROPORTION_COEFFICIENT);
        m_intakeAdjustMotor.config_kD(Constants.PIDConstants.PID_ID, Constants.PIDConstants.DERIVATIVE_COEFFICEINT);
        m_intakeAdjustMotor.config_kI(Constants.PIDConstants.PID_ID, Constants.PIDConstants.INTEGRAL_COEFFICIENT);
    
    }

    public void setIntakeOn()
    {
        m_intakeMotor.set(ControlMode.PercentOutput, Constants.IntakeConstants.INTAKE_MOTOR_POWER);
    }

    public void setIntakeOff()
    {
        m_intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    public void getPosition()
    {
        SmartDashboard.putNumber("Selected sensor position: ", m_intakeAdjustMotor.getSelectedSensorPosition());
    }

    /*
     * public void setArmPosition(double position) { //double demand = position *
     * Constants.PIDConstants.POSITION_MULTIPLIER * (double)Constants.PIDConstants.ROTATION_TICKS; double demand =
     * position * 4096; m_intakeAdjustMotor.set(ControlMode.Position, demand);
     * SmartDashboard.putNumber("intake positon: ", demand); }
     */

    public void armPositionTest()
    {
        //double targetRotation = 5 * 4096;
        m_intakeAdjustMotor.set(ControlMode.Position, 8000);
    }

    public void armPositionTestStop()
    {
        m_intakeAdjustMotor.set(ControlMode.PercentOutput, 0);
    }

    public void periodic()
    {

        SmartDashboard.putNumber("tgt: ", m_intakeAdjustMotor.getClosedLoopTarget());
        SmartDashboard.putNumber("err: ", m_intakeAdjustMotor.getErrorDerivative());
        getPosition();
    }

    public void printPDPCurrent()
    {
        double currentIntakeMotor = m_intakeMotor.getSupplyCurrent();
        SmartDashboard.putNumber("MOTOR_CONTROLLER_INTAKE: ", currentIntakeMotor);
    }

    public double getSetpoint()
    {
        return Constants.IntakeConstants.ARM_DOWN_ANGLE;
    }

    

    public void intakePrinter()
    {
        //SmartDashboard.putNumber("Arm setpoint: ", PID.getSetpoint());
        SmartDashboard.putNumber("Potentiometer reading: ", getPotentiometerReading());
    }

}
