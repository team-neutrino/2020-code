/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterSetSpeedCommand extends CommandBase
{
    private ShooterSubsystem m_shooter;
    private ShuffleboardTab smartdashboard = Shuffleboard.getTab("SmartDashboard");
    private NetworkTableEntry velocity =
        smartdashboard.add("Shooter Velocity", 80000).getEntry();
    /**
     * Creates a new ShooterSetSpeedCommand.
     */
    public ShooterSetSpeedCommand(ShooterSubsystem p_shooter)
    {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(p_shooter);
        m_shooter = p_shooter;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        m_shooter.setVelocity(velocity.getDouble(80000));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        m_shooter.setPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
