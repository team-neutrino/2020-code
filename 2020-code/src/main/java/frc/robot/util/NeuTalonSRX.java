// made this wrapper because TalonSRX doesn't support sim mode

package frc.robot.util;

import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Victor;

//==============================================================================
public class NeuTalonSRX
{
    private TalonSRX m_TalonSRX;
    private Victor m_simTalonSRX;
    static boolean m_sim = Robot.isSimulation();

    //==========================================================================
    public NeuTalonSRX(int deviceNumber)
    {
        if (m_sim)
        {
            m_simTalonSRX = new Victor(deviceNumber);
        }
        else
        {
            m_TalonSRX = new TalonSRX(deviceNumber);
        }
    }

    //==========================================================================
    public void setPercentOutput(double percent)
    {
        if (m_sim)
        {
            m_simTalonSRX.set(percent);
        }
        else
        {
            m_TalonSRX.set(ControlMode.PercentOutput, percent);
        }
    }

    //==========================================================================
    public double getPercentOutput()
    {
        if (m_sim)
        {
            if (m_simTalonSRX.getInverted())
            {
                return -m_simTalonSRX.get();
            }
            return m_simTalonSRX.get();
        }
        else
        {
            return m_TalonSRX.getMotorOutputPercent();
        }
    }

    //==========================================================================
    public void setInverted(boolean invert)
    {
        if (m_sim)
        {
            m_simTalonSRX.setInverted(invert);
        }
        else
        {
            m_TalonSRX.setInverted(invert);
        }
    }

    //==========================================================================
    public void close()
    {
        if (m_sim)
        {
            m_simTalonSRX.close();
        }
    }
}
