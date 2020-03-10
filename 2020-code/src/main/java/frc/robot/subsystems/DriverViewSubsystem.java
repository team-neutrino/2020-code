/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;
import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriverViewSubsystem extends SubsystemBase
{
    /**
     * Creates a new DriverViewSubsystem.
     */
    private ShooterSubsystem m_Shooter;
    private TurretSubsystem m_Turret;
    private HopperSubsystem m_Hopper;

    private ShuffleboardTab m_driver_view_tab;
    private NetworkTableEntry m_shooter_velocity;
    private NetworkTableEntry m_turret_angle;
    private NetworkTableEntry m_beam_break_top;
    private NetworkTableEntry m_beam_break_bot;

    public DriverViewSubsystem(ShooterSubsystem p_Shooter, TurretSubsystem p_Turret, HopperSubsystem p_Hopper)
    {
        m_Shooter = p_Shooter;
        m_Turret = p_Turret;
        m_Hopper = p_Hopper;

        // setup driver view tab
        m_driver_view_tab = Shuffleboard.getTab("Driver View");
        m_shooter_velocity = m_driver_view_tab.add("Shooter Velocity", 0).withPosition(1, 0).withSize(2, 2).withWidget(
            BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 120000)).getEntry();
        m_turret_angle = m_driver_view_tab.add("Turret Angle", 0).withWidget(BuiltInWidgets.kDial).withPosition(0,
            2).withSize(2, 2).withProperties(Map.of("min", -180, "max", 180)).getEntry();
        m_beam_break_top = m_driver_view_tab.add("Top Beam Status", false).withPosition(0, 0).getEntry();
        m_beam_break_bot = m_driver_view_tab.add("Bottom Beam Status", false).withPosition(0, 1).getEntry();

        HttpCamera limelightFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg",
            HttpCameraKind.kMJPGStreamer);
        CameraServer.getInstance().startAutomaticCapture(limelightFeed);
        m_driver_view_tab.add(limelightFeed);
    }

    @Override
    public void periodic()
    {
        m_shooter_velocity.setDouble(m_Shooter.getVelocity());
        m_turret_angle.setDouble(m_Turret.getTurretAngle());
        m_beam_break_top.setBoolean(m_Hopper.topBeamStatus());
        m_beam_break_bot.setBoolean(m_Hopper.bottomBeamStatus());
    }
}
