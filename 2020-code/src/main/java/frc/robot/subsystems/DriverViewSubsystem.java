/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Limelight;

public class DriverViewSubsystem extends SubsystemBase
{
    /**
     * Creates a new DriverViewSubsystem.
     */
    private ShooterSubsystem m_Shooter;
    private ShuffleboardTab tab = Shuffleboard.getTab("Driver View");
    private TurretSubsystem m_Turret;
    private HopperSubsystem m_Hopper;
    private VideoSource m_Limelight;

    private NetworkTableEntry m_shooter_velocity = tab.add("Shooter Velocity", 0).getEntry();
    private NetworkTableEntry m_turret_angle = tab.add("Turret Angle", 0).getEntry();
    private NetworkTableEntry m_beam_break_bot = tab.add("Bottom Beam Status", 0).getEntry();
    private NetworkTableEntry m_beam_break_top = tab.add("Top Beam Status", 0).getEntry();

    public DriverViewSubsystem(ShooterSubsystem p_Shooter, TurretSubsystem p_Turret, HopperSubsystem p_Hopper)
    {
        m_Shooter = p_Shooter;
        m_Turret = p_Turret;
        m_Hopper = p_Hopper;

        HttpCamera limelightFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg",
            HttpCameraKind.kMJPGStreamer);
        // HttpCamera limelightFeed = new HttpCamera("limelight", "http://10.3.22.11:5800/stream.mjpg", HttpCameraKind.kMJPGStreamer);
        CameraServer.getInstance().startAutomaticCapture(limelightFeed);
        // m_Limelight = CameraServer.getInstance().getVideo("limelight").getSource();
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        m_shooter_velocity.setDouble(m_Shooter.getVelocity());
        m_turret_angle.setDouble(m_Turret.getTurretAngle());
        m_beam_break_bot.setBoolean(m_Hopper.bottomBeamStatus());
        m_beam_break_top.setBoolean(m_Hopper.topBeamStatus());
        // tab.add(m_Limelight);
    }
}
