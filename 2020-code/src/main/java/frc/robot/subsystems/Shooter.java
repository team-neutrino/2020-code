/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings({"all"})
public class Shooter extends SubsystemBase {

  private TalonSRX motor;
  private PIDController wheelPid;
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    motor = new TalonSRX(Constants.ShooterConstants.WheelMotorPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
