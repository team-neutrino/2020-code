// made this wrapper because TalonSRX doesn't support sim mode very well

package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class NeuTalonSRX extends TalonSRX
{
    protected double m_output_percent;

    public NeuTalonSRX(int deviceNumber)
    {
        super(deviceNumber);
        m_output_percent = 0;
    }

    public void setPercentOutput(double percent)
    {
        m_output_percent = percent;
        super.set(ControlMode.PercentOutput, m_output_percent);
    }

    public double getPercentOutput()
    {
        return m_output_percent;
    }
}
