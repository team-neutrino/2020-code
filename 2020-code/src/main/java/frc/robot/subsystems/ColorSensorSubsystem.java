/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensorSubsystem extends SubsystemBase {
  /**
   * Creates a new ColorSensorSubsystem.
   */
  public DigitalInput m_colorSensor = new DigitalInput(8);
  public ColorSensorSubsystem() 
  {

  }

  @Override
  public void periodic() 
  {
      // This method will be called once per scheduler run
      SmartDashboard.putBoolean("Color Sensor", m_colorSensor.get());
  
  }
}
