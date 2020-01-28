/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class IntakeSetPositionPIDCommand extends PIDCommand {
  /**
   * Creates a new IntakeSetPositionPIDCommand.
   */
  public IntakeSetPositionPIDCommand(IntakeSubsystem m_intake) {
    super(
        // The controller that the command will use
        new PIDController(Constants.PIDConstants.PROPORTION_COEFFICIENT, 
                        Constants.PIDConstants.INTEGRAL_COEFFICIENT, 
                        Constants.PIDConstants.DERIVATIVE_COEFFICEINT),
        // This should return the measurement
        () -> m_intake.getPotentiometerReading(),
        // This should return the setpoint (can also be a constant)
        () -> m_intake.getIntakeSetpoint(),
        // This uses the output
        output -> m_intake.setIntakeAngle(Constants.IntakeConstants.ARM_DOWN_ANGLE));
          // Use the output here
      
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
