/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings({"all"})
public class ShooterSubsystem extends SubsystemBase {

  private TalonSRX m_wheelMotor;
  //TODO find out what kind of encoder we are using
  private Encoder m_wheelEncoder;
  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(Constants.ShooterConstants.KS_VOLTS,
  Constants.ShooterConstants.KV_VOLT_SEC_PER_ROTATION);
  /**
   * Creates a new Shooter.
   */


  public ShooterSubsystem() 
  {
    m_wheelMotor = new TalonSRX(Constants.ShooterConstants.WHEEL_MOTOR_PORT);
    m_wheelEncoder = new Encoder(Constants.ShooterConstants.WHEEL_ENCODER_PORT_1,
      Constants.ShooterConstants.WHEEL_ENCODER_PORT_2);
    m_wheelEncoder.setDistancePerPulse(Constants.ShooterConstants.WHEEL_ENCODER_DIST_PER_PULSE);
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }

  public double setpoint()
  {
    return 0;
  }

  public double getWheelEncoderDistance()
  {
    return m_wheelEncoder.getRate();
  }

  public void setWheelMotor(double demand)
  {
    m_wheelMotor.set(ControlMode.PercentOutput, demand);
  }
}