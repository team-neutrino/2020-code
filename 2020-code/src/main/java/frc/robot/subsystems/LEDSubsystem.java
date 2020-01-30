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

public class LEDSubsystem extends SubsystemBase
{
    AddressableLED led = new AddressableLED(0);
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(17);

    /**
     * Creates a new LED.
     */
    public LEDSubsystem()
    {
        led.setLength(17);
        for (int i = 0; i < 17; i++)
        {
            buffer.setRGB(i, 200, 25, 10);
        }
        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic()
    {
    }
}
