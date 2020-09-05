/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;
import frc.robot.util.NeuTalonSRX;

public class HopperSubsystem extends SubsystemBase
{
    /**
     * Creates a new HopperSubsystem.
     */
    private DigitalInput m_beamBreakTop = new DigitalInput(HopperConstants.HOPPER_TOP_BEAMBREAK);
    private DigitalInput m_beamBreakBot = new DigitalInput(HopperConstants.HOPPER_BOT_BEAMBREAK);
    private NeuTalonSRX m_towerMotor = new NeuTalonSRX(Constants.CanId.MOTOR_CONTROLLER_TOWER);
    private NeuTalonSRX m_intakeHopperMotor = new NeuTalonSRX(Constants.CanId.MOTOR_CONTROLLER_HOPPER);
    private Timer m_timer = new Timer();
    private Timer m_rollerTimer = new Timer();
    private boolean m_prevBotBeam;
    private ShooterSubsystem m_Shooter;

    public HopperSubsystem(ShooterSubsystem p_Shooter)
    {
        m_rollerTimer.start();
        m_towerMotor.setInverted(true);
        m_intakeHopperMotor.setInverted(false);
        m_timer.reset();
        m_Shooter = p_Shooter;
    }

    //used when not shooting will run until ball is at top and ready
    public void towerIndexing()
    {
        m_towerMotor.setPercentOutput(0.5);
        m_intakeHopperMotor.setPercentOutput(HopperConstants.HOPPER_MOTOR_POWER);
    }

    //used when shooting
    public void towerShoot()
    {
        m_towerMotor.setPercentOutput(1);
        m_intakeHopperMotor.setPercentOutput(HopperConstants.HOPPER_MOTOR_POWER);
    }

    public void conditionalTowerShoot()
    {
        if (m_Shooter.getVelocity() > 60000)
        {
            m_towerMotor.setPercentOutput(1);
        }
        else
        {
            m_towerMotor.setPercentOutput(0);
        }
    }

    public void reverse()
    {
        m_towerMotor.setPercentOutput(HopperConstants.HOPPER_MOTOR_POWER_REVERSE);
    }

    public void stop()
    {
        m_towerMotor.setPercentOutput(0);
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

    public void rollerTowardsIntake()
    {
        m_intakeHopperMotor.setPercentOutput(0.3);
    }

    public void rollerTowardsTower()
    {
        m_intakeHopperMotor.setPercentOutput(-0.3);
    }

    @Override
    public void periodic()
    {
        rollerTowardsIntake();
        /*
         * if (m_rollerTimer.get() < 1) { rollerTowardsIntake(); } else { rollerTowardsTower(); } if
         * (m_rollerTimer.get() > 2) { m_rollerTimer.reset(); }
         */
    }

    // release HAL resources
    public void close()
    {
        m_beamBreakTop.close();
        m_beamBreakBot.close();
    }

    // set the intake motor object
    public void SetIntakeMotor(NeuTalonSRX motor)
    {
        m_intakeHopperMotor = motor;
    }

    // set the tower motor object
    public void SetTowerMotor(NeuTalonSRX motor)
    {
        m_towerMotor = motor;
    }

    public double GetTowerMotorPercentOutput()
    {
        return m_towerMotor.getPercentOutput();
    }

    public double GetIntakeMotorPercentOutput()
    {
        return m_intakeHopperMotor.getPercentOutput();
    }
}
