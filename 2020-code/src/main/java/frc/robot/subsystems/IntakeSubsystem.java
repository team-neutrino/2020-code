
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
    private AnalogPotentiometer m_adjustMotorPotentiometer = new AnalogPotentiometer(
        Constants.IntakeConstants.ADJUST_MOTOR_ENCODER);
    private TalonSRX m_intakeMotor = new TalonSRX(CanId.MOTOR_CONTROLLER_INTAKE);
    private TalonSRX m_intakeAdjustMotor = new TalonSRX(CanId.MOTOR_CONTROLLER_INTAKE_ADJUST);

    public IntakeSubsystem()
    {
        m_intakeAdjustMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        m_intakeAdjustMotor.config_kP(Constants.PIDConstants.PID_ID, Constants.PIDConstants.PROPORTION_COEFFICIENT);
        m_intakeAdjustMotor.config_kD(Constants.PIDConstants.PID_ID, Constants.PIDConstants.DERIVATIVE_COEFFICEINT);
        m_intakeAdjustMotor.config_kI(Constants.PIDConstants.PID_ID, Constants.PIDConstants.INTEGRAL_COEFFICIENT);
    }

    public void periodic()
    {

        SmartDashboard.putNumber("tgt: ", m_intakeAdjustMotor.getClosedLoopTarget());
        SmartDashboard.putNumber("err: ", m_intakeAdjustMotor.getClosedLoopError());
        SmartDashboard.putNumber("Intake motor current: ", m_intakeMotor.getSupplyCurrent());
        SmartDashboard.putNumber("Arm motor current: ", m_intakeAdjustMotor.getSupplyCurrent());
    }

    public void setIntakeOn()
    {
        m_intakeMotor.set(ControlMode.PercentOutput, Constants.IntakeConstants.INTAKE_MOTOR_POWER);
    }

    public void setIntakeOff()
    {
        m_intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    public int yButtonCounter = 0;
    public void increaseYButtonCounter()
    {
        yButtonCounter++;
    }

    public void setArmDown()
    {
        double DOWN_DEMAND = Constants.IntakeConstants.ARM_DOWN_ANGLE
                * Constants.IntakeConstants.ENCODER_PULSES_PER_REV;
        m_intakeAdjustMotor.set(ControlMode.Position, DOWN_DEMAND);
    }

    public void setArmUp()
    {
        double UP_DEMAND = Constants.IntakeConstants.ARM_UP_ANGLE * Constants.IntakeConstants.ENCODER_PULSES_PER_REV;
        m_intakeAdjustMotor.set(ControlMode.Position, UP_DEMAND);
    }

    /*
     * public void printCurrent() { SmartDashboard.putNumber("MOTOR_CONTROLLER_INTAKE: ", currentIntakeMotor); } public
     * void printPotentiometer() { //SmartDashboard.putNumber("Arm setpoint: ", PID.getSetpoint());
     * SmartDashboard.putNumber("Potentiometer reading: ", getPotentiometerReading()); }
     */

}