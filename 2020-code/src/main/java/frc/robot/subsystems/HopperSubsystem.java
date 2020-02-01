/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase
{
    /**
     * Creates a new HopperSubsystem.
     */
    public DigitalInput m_beamBreakTop = new DigitalInput(8);
    public DigitalInput m_beamBreakBot = new DigitalInput(7);
    private TalonSRX m_hopperMotor = new TalonSRX(HopperConstants.MOTOR_CONTROLLER_HOPPER); 

    public HopperSubsystem()
    {

    }

    public void intake()
    {
        m_hopperMotor.set(ControlMode.PercentOutput, HopperConstants.HOPPER_MOTOR_POWER);
    }

    public void reverse()
    {
        m_hopperMotor.set(ControlMode.PercentOutput, HopperConstants.HOPPER_MOTOR_POWER_REVERSE);
    }

    public void stop()
    {
        m_hopperMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        if (m_beamBreakTop.get() == false && m_beamBreakBot.get() == true)
        {
            m_hopperMotor.set(ControlMode.PercentOutput, HopperConstants.HOPPER_MOTOR_POWER);
        }
        else if ((m_beamBreakTop.get() == true) || m_beamBreakBot.get() == false)
        {
            m_hopperMotor.set(ControlMode.PercentOutput, 0);
        }
        System.out.print(ControlMode.PercentOutput);
    }

}
