/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriverViewSubsystem extends SubsystemBase {
  /**
   * Creates a new DriverViewSubsystem.
   */
  private ShooterSubsystem m_Shooter;
  private ShuffleboardTab tab;
  
  public DriverViewSubsystem()
  {
    tab = Shuffleboard.getTab("Driver View");
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    tab.add("Shooter Velocity", m_Shooter.getVelocity());
  }
}
