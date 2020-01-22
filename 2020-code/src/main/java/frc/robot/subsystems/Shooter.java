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
public class Shooter extends SubsystemBase {

  private TalonSRX wheelMotor;
  //TODO find out what kind of encoder we are using
  private Encoder wheelEncoder;
  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(Constants.ShooterConstants.kSVolts,
  Constants.ShooterConstants.kVVoltSecondsPerRotation);
  /**
   * Creates a new Shooter.
   */


  public Shooter() 
  {
    wheelMotor = new TalonSRX(Constants.ShooterConstants.WheelMotorPort);
    wheelEncoder = new Encoder(Constants.ShooterConstants.WheelEncoderPort1,
    Constants.ShooterConstants.WheelEncoderPort2);
    wheelEncoder.setDistancePerPulse(Constants.ShooterConstants.WheelEncoderDistancePerPulse);
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
    return wheelEncoder.getRate();
  }

  public TalonSRX getWheelMotor()
  {
    return wheelMotor;
  }

  public void setWheelMotor(double demand)
  {
    wheelMotor.set(ControlMode.PercentOutput, demand);
  }
}
