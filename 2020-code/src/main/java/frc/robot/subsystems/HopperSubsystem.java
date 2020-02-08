/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase
{
    /**
     * Creates a new HopperSubsystem.
     */
    public DigitalInput m_beamBreakTop = new DigitalInput(HopperConstants.HOPPER_TOP_BEAMBREAK);
    public DigitalInput m_beamBreakBot = new DigitalInput(HopperConstants.HOPPER_BOT_BEAMBREAK);
    private TalonSRX m_hopperMotor = new TalonSRX(Constants.CanId.MOTOR_CONTROLLER_HOPPER);

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
        SmartDashboard.putBoolean("Beam Break 1", m_beamBreakBot.get());
        SmartDashboard.putBoolean("Beam Break 2", m_beamBreakTop.get());
        SmartDashboard.putBoolean("Beam Break 3", m_beamBreakTop.get() == false && m_beamBreakBot.get() == true);
        // This method will be called once per scheduler run
        if (m_beamBreakTop.get() == false && m_beamBreakBot.get() == true)
        {
            intake();
        }
        else
        {
            stop();
        }
        System.out.print(ControlMode.PercentOutput);

    }

}
