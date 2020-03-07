/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Limelight;

public class DriverViewSubsystem extends SubsystemBase
{
    /**
     * Creates a new DriverViewSubsystem.
     */
    private ShooterSubsystem m_Shooter;
    private ShuffleboardTab tab;
    private TurretSubsystem m_Turret;
    private HopperSubsystem m_Hopper;

    public DriverViewSubsystem(ShooterSubsystem p_Shooter, TurretSubsystem p_Turret, HopperSubsystem p_Hopper)
    {
        tab = Shuffleboard.getTab("Driver View");
        m_Shooter = p_Shooter;
        m_Turret = p_Turret;
        m_Hopper = p_Hopper;
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        tab.add("Shooter Velocity", m_Shooter.getVelocity());
        tab.add("Turret Angle", m_Turret.getTurretAngle());
        tab.add("Bottom Beam Status", m_Hopper.bottomBeamStatus());
        tab.add("Top Beam Status", m_Hopper.topBeamStatus());
        tab.add("Limelight", CameraServer.getInstance().getVideo("limelight"));
    }
}
