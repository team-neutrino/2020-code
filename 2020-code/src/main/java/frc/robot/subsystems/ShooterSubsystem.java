/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings(
{ "all" })
public class ShooterSubsystem extends SubsystemBase
{

    private TalonSRX m_wheelMotor;
    private TalonSRX m_wheelMotor2;
    private TalonSRX m_wheelMotor3;
    //TODO find out what kind of encoder we are using
    private Encoder m_wheelEncoder;
    private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(
        Constants.ShooterConstants.KS_VOLTS, Constants.ShooterConstants.KV_VOLT_SEC_PER_ROTATION);

    /**
     * Creates a new Shooter.
     */

    public ShooterSubsystem()
    {

        m_wheelMotor = new TalonSRX(Constants.CanId.MOTOR_CONTROLLER_SHOOTERWHEEL);
        m_wheelMotor2 = new TalonSRX(Constants.CanId.MOTOR_CONTROLLER_SHOOTERWHEEL2);
        m_wheelMotor3 = new TalonSRX(Constants.CanId.MOTOR_CONTROLLER_SHOOTERWHEEL3);
        m_wheelMotor2.follow(m_wheelMotor);
        m_wheelMotor3.follow(m_wheelMotor);
        m_wheelEncoder = new Encoder(Constants.ShooterConstants.WHEEL_ENCODER_PORT_1,
            Constants.ShooterConstants.WHEEL_ENCODER_PORT_2);
        m_wheelEncoder.setDistancePerPulse(Constants.ShooterConstants.WHEEL_ENCODER_DIST_PER_PULSE);
        m_wheelMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        m_wheelMotor.configFactoryDefault();
        m_wheelMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
            Constants.PIDConstants.kPIDLoopIdx, Constants.PIDConstants.kTimeoutMs);
        m_wheelMotor.setSensorPhase(true);
        m_wheelMotor.configNominalOutputForward(0, Constants.PIDConstants.kTimeoutMs);
        m_wheelMotor.configNominalOutputReverse(0, Constants.PIDConstants.kTimeoutMs);
        m_wheelMotor.configPeakOutputForward(1, Constants.PIDConstants.kTimeoutMs);
        m_wheelMotor.configPeakOutputReverse(-1, Constants.PIDConstants.kTimeoutMs);
        m_wheelMotor.config_kF(Constants.PIDConstants.kPIDLoopIdx, Constants.PIDConstants.kF,
            Constants.PIDConstants.kTimeoutMs);
        m_wheelMotor.config_kP(Constants.PIDConstants.kPIDLoopIdx, Constants.PIDConstants.kP,
            Constants.PIDConstants.kTimeoutMs);
        m_wheelMotor.config_kI(Constants.PIDConstants.kPIDLoopIdx, Constants.PIDConstants.kI,
            Constants.PIDConstants.kTimeoutMs);
        m_wheelMotor.config_kD(Constants.PIDConstants.kPIDLoopIdx, Constants.PIDConstants.kD,
            Constants.PIDConstants.kTimeoutMs);

    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("speed",
            Constants.ShooterConstants.RPMC_CONVERTED_VALUE * m_wheelMotor.getSelectedSensorVelocity());
        // This method will be called once per scheduler run
    }

    public double setpoint()
    {
        return 1;
    }

    public double getWheelEncoderDistance()
    {
        return m_wheelEncoder.getRate();
    }

    public void setWheelMotor(double power)
    {
        m_wheelMotor.set(ControlMode.PercentOutput, power);

    }

    public boolean getMotorSpeedStatus()
    {
        return false;
    }
}
