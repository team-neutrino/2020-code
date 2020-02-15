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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase
{
    /**
     * Creates a new HopperSubsystem.
     */
    private DigitalInput m_beamBreakTop = new DigitalInput(HopperConstants.HOPPER_TOP_BEAMBREAK);
    private DigitalInput m_beamBreakBot = new DigitalInput(HopperConstants.HOPPER_BOT_BEAMBREAK);
    private TalonSRX m_towerMotor = new TalonSRX(Constants.CanId.MOTOR_CONTROLLER_TOWER);
    private TalonSRX m_intakeHopperMotor = new TalonSRX(Constants.CanId.MOTOR_CONTROLLER_HOPPER);
    private Timer m_timer = new Timer();
    private boolean m_prevBotBeam;

    public HopperSubsystem()
    {
        m_towerMotor.setInverted(true);
        m_intakeHopperMotor.setInverted(true);
        m_timer.reset();
    }

    public void intake()
    {
        m_towerMotor.set(ControlMode.PercentOutput, 0.7);
        m_intakeHopperMotor.set(ControlMode.PercentOutput, HopperConstants.HOPPER_MOTOR_POWER);

    }

    public void reverse()
    {
        m_towerMotor.set(ControlMode.PercentOutput, HopperConstants.HOPPER_MOTOR_POWER_REVERSE);
    }

    public void stop()
    {
        m_towerMotor.set(ControlMode.PercentOutput, 0);
        m_timer.stop();
        m_timer.reset();
    }

    public double getTime()
    {
        return m_timer.get();
    }

    public void startTimer()
    {
        m_timer.start();
    }

    public void stopTimer()
    {
        m_timer.stop();
    }

    public void resetTimer()
    {
        m_timer.reset();
    }

    public void setPrevBotBeam(boolean beamStatus)
    {
        m_prevBotBeam = beamStatus;
    }

    public boolean getPrevBotBeam()
    {
        return m_prevBotBeam;
    }

    public boolean bottomBeamStatus()
    {
        return m_beamBreakBot.get();
    }

    public boolean topBeamStatus()
    {
        return m_beamBreakTop.get();
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("Beam Break 1", m_beamBreakBot.get());
        SmartDashboard.putBoolean("Beam Break 2", m_beamBreakTop.get());
        m_intakeHopperMotor.set(ControlMode.PercentOutput, 0.3);
    }

}
