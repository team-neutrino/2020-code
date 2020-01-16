/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

@SuppressWarnings({"all"})
public class ShooterPIDSubsystem extends CommandBase {
  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(Constants.ShooterConstants.kSVolts,
  Constants.ShooterConstants.kVVoltSecondsPerRotation);
  private final Encoder wheelEncoder = new Encoder(Constants.ShooterConstants.WheelEncoderPort1,
    Constants.ShooterConstants.WheelEncoderPort2);
  /**
   * Creates a new ShooterPIDSubsystem.
   */
  public ShooterPIDSubsystem() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
