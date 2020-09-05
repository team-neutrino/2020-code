
package frc.robot.util.NeuMotor;

// CAN Spark Max
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//******************************************************************************
class NeuCANSparkMax implements NeuMotorIf
{
    private CANSparkMax m_CANSparkMax;

    //**************************************************************************
    // assumed to be a brushless config
    public NeuCANSparkMax(int deviceNumber)
    {
        m_CANSparkMax = new CANSparkMax(deviceNumber, MotorType.kBrushless);
    }

    //**************************************************************************
    public void set(NeuMotorMode mode, double outputValue)
    {
        switch (mode)
        {
            case Percent:
                m_CANSparkMax.set(outputValue);
                break;
            default:
                return;
        }
    }

    //**************************************************************************
    public void set(double outputValue)
    {
        set(NeuMotorMode.Percent, outputValue);
    }

    //**************************************************************************
    public double get()
    {
        return m_CANSparkMax.get();
    }

    //**************************************************************************
    public void setInverted(boolean invert)
    {
        m_CANSparkMax.setInverted(invert);
    }

    //**************************************************************************
    public void close()
    {
    }
}