// made this wrapper because TalonSRX doesn't support sim mode very well

package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class NeuTalonSRX
{
    protected double m_output_percent;
    private TalonSRX m_TalonSRX;

    public NeuTalonSRX(int deviceNumber)
    {
        m_TalonSRX = new TalonSRX(deviceNumber);
        m_output_percent = 0;
    }

    public void setPercentOutput(double percent)
    {
        m_output_percent = percent;
        m_TalonSRX.set(ControlMode.PercentOutput, m_output_percent);
    }

    public double getPercentOutput()
    {
        return m_output_percent;
    }

    public void setInverted( boolean invert )
    {
        m_TalonSRX.setInverted(invert);
    }
}
