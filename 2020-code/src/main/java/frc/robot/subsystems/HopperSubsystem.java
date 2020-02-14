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
    private DigitalInput m_beamBreakTop = new DigitalInput(HopperConstants.HOPPER_TOP_BEAMBREAK);
    private DigitalInput m_beamBreakBot = new DigitalInput(HopperConstants.HOPPER_BOT_BEAMBREAK);
<<<<<<< HEAD
    private TalonSRX m_towerMotor = new TalonSRX(Constants.CanId.MOTOR_CONTROLLER_TOWER);
=======
    private TalonSRX m_hopperMotor = new TalonSRX(Constants.CanId.MOTOR_CONTROLLER_TOWER);
>>>>>>> master
    private TalonSRX m_intakeHopperMotor = new TalonSRX(Constants.CanId.MOTOR_CONTROLLER_HOPPER);

    public HopperSubsystem()
    {
        m_towerMotor.setInverted(true);
    }

    public void intake()
    {
<<<<<<< HEAD
        m_towerMotor.set(ControlMode.PercentOutput, HopperConstants.HOPPER_MOTOR_POWER);
=======
        m_hopperMotor.set(ControlMode.PercentOutput, HopperConstants.HOPPER_MOTOR_POWER);
>>>>>>> master
        m_intakeHopperMotor.set(ControlMode.PercentOutput, HopperConstants.HOPPER_MOTOR_POWER);

    }

    public void reverse()
    {
        m_towerMotor.set(ControlMode.PercentOutput, HopperConstants.HOPPER_MOTOR_POWER_REVERSE);
    }

    public void stop()
    {
        m_towerMotor.set(ControlMode.PercentOutput, 0);
        m_intakeHopperMotor.set(ControlMode.PercentOutput, 0);

    }

    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("Beam Break 1", m_beamBreakBot.get());
        SmartDashboard.putBoolean("Beam Break 2", m_beamBreakTop.get());
        // This method will be called once per scheduler run
        if (m_beamBreakTop.get() == true && m_beamBreakBot.get() == false)
        {
            intake();
        }
        else
        {
            stop();
        }

    }

}
