/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

@SuppressWarnings(
{ "all" })
public class ShooterSubsystem extends SubsystemBase
{
    private TalonSRX m_wheelMotor;
    private TalonSRX m_wheelMotor2;
    private TalonSRX m_wheelMotor3;
    private Encoder m_wheelEncoder;
    

    /**
     * Creates a new Shooter.
     */

    public ShooterSubsystem()
    {
        conifgSRX(); 
        m_wheelMotor = new TalonSRX(Constants.CanId.MOTOR_CONTROLLER_SHOOTERWHEEL);
        m_wheelMotor2 = new TalonSRX(Constants.CanId.MOTOR_CONTROLLER_SHOOTERWHEEL2);
        m_wheelMotor3 = new TalonSRX(Constants.CanId.MOTOR_CONTROLLER_SHOOTERWHEEL3);

        m_wheelMotor.configAllSettings(ShooterConstants.WHEEL_MASTER_CONFIGURATION);
        m_wheelMotor2.configAllSettings(ShooterConstants.WHEEL_FOLLOWER_CONFIGURATION);
        m_wheelMotor3.configAllSettings(ShooterConstants.WHEEL_FOLLOWER_CONFIGURATION);
        m_wheelMotor2.follow(m_wheelMotor);
        m_wheelMotor3.follow(m_wheelMotor);

        m_wheelMotor.setInverted(true);
        m_wheelMotor2.setInverted(true);
        m_wheelMotor3.setInverted(true);

        m_wheelMotor.setNeutralMode(NeutralMode.Coast);
        m_wheelMotor2.setNeutralMode(NeutralMode.Coast);
        m_wheelMotor3.setNeutralMode(NeutralMode.Coast);


        m_wheelEncoder = new Encoder(ShooterConstants.WHEEL_ENCODER_PORT_1,
        ShooterConstants.WHEEL_ENCODER_PORT_2);
    }

    @Override
    public void periodic()
    {
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

    public void setPower(double power)
    {
        m_wheelMotor.set(ControlMode.PercentOutput, power);
    }

    public void setSpeed(double speed)
    {
        m_wheelMotor.set(ControlMode.Velocity, speed);
    }

    public boolean getMotorSpeedStatus()
    {
        return false;
    }

    private void conifgSRX()
    {
        ShooterConstants.WHEEL_MASTER_CONFIGURATION.slot0.kP = ShooterConstants.WHEEL_P;
        ShooterConstants.WHEEL_MASTER_CONFIGURATION.slot0.kI = ShooterConstants.WHEEL_I;
        ShooterConstants.WHEEL_MASTER_CONFIGURATION.slot0.kD = ShooterConstants.WHEEL_D;
        ShooterConstants.WHEEL_MASTER_CONFIGURATION.slot0.kF = ShooterConstants.WHEEL_F;
    }

}
