/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public final class JoystickProcessor {
    /**
     * Applies deadzoning and curve to the joystick input
     *
     * @return A processed joystick input
     */
    private JoystickProcessor()
    {
        System.out.println("If you got here, you did something wrong.");
    }
    
    public static double process(double input)
    {
        if (Math.abs(input) > Constants.JoystickConstants.DEADZONE_SIZE)
        {
            double absoluteValue = Math.abs(input);
            double deadzoneCorrectedAbsoluteValue = (1 / (1 - Constants.JoystickConstants.DEADZONE_SIZE))
                    * (absoluteValue - 1.0) + 1.0;
            return Math.pow(deadzoneCorrectedAbsoluteValue, Constants.JoystickConstants.JOYSTICK_CURVE)
                    * (absoluteValue / input);
        }
        else
        {
            return 0.0;
        }
    }
}
