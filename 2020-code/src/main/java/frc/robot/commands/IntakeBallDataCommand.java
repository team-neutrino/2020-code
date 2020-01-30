/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeBallDataCommand extends CommandBase {
  IntakeSubsystem m_intake;
  /**
   * Creates a new IntakeBallDataCommand.
   */
  public IntakeBallDataCommand(IntakeSubsystem p_intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(p_intake);
    m_intake = p_intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.getPDPCurrent();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_intake.setIntakeOn();
      m_intake.getPDPCurrent();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.getPDPCurrent();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  
  }
}
