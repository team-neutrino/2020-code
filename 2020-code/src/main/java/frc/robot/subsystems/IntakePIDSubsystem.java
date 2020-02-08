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

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import frc.robot.Constants;
import frc.robot.Constants.CanId;

public class IntakePIDSubsystem extends PIDSubsystem {
  private TalonSRX m_IntakeFeedMotor = new TalonSRX(CanId.MOTOR_CONTROLLER_INTAKE_FEED);
  private TalonSRX m_IntakePositionMotor = new TalonSRX(CanId.MOTOR_CONTROLLER_INTAKE_POSITION);
  private DutyCycleEncoder m_DutyCycleEncoder = new DutyCycleEncoder(Constants.IntakeConstants.ENCODER_PORT);
  
  /**
   * Creates a new IntakePIDSubsystem.
   */
  public IntakePIDSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.PIDConstants.PROPORTION_COEFFICIENT,
                          Constants.PIDConstants.INTEGRAL_COEFFICIENT, 
                          Constants.PIDConstants.DERIVATIVE_COEFFICEINT));
  }

  @Override
  public void useOutput(double output, double setpoint) {
    m_IntakePositionMotor.set(ControlMode.PercentOutput, output);
    SmartDashboard.putNumber("IntakePIDSubsystem setpoint", setpoint);
  }

  @Override
  public double getMeasurement() {
    return m_DutyCycleEncoder.get();
  }

  /**
   * angle in degrees
   * @param angle
   */
  public void setAngle(double angle) 
  {
    setSetpoint(angle);
  }

  public void setIntakeOn()
  {
    m_IntakeFeedMotor.set(ControlMode.PercentOutput, 1);
  }

  public void setIntakeOff()
  {
    m_IntakeFeedMotor.set(ControlMode.PercentOutput, 0);
  }

}
