/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  AddressableLED led=new AddressableLED(0);
  AddressableLEDBuffer buffer = new AddressableLEDBuffer(1);
  /**
   * Creates a new LED.
   */
  public LED() {
led.setLength(17);

  }

  @Override
  public void periodic() {
    buffer.setRGB(0, 32, 32, 32);
    led.setData(buffer);
    led.start();
  }
}
