/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class IntakeRetractCommand extends ParallelCommandGroup
{
    /**
    * Creates a new IntakeRetractCommand.
    */
    public IntakeRetractCommand(IntakeSubsystem m_Intake)
    {
        super(
            new InstantCommand(() -> m_Intake.setIntake(false)),
            new InstantCommand(() -> m_Intake.setPusherIn())
        );
    }
}
