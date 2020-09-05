
package frc.robot.util.NeuMotor;

// PWM Victor
import edu.wpi.first.wpilibj.Victor;

//******************************************************************************
class NeuSimMotorController implements NeuMotorIf
{
    // we use a PWM Victor in simulation
    private Victor m_sim_victor;

    //**************************************************************************
    public NeuSimMotorController(int deviceNumber)
    {
        m_sim_victor = new Victor(deviceNumber);
    }

    //**************************************************************************
    public void set(NeuMotorMode mode, double outputValue)
    {
        switch (mode)
        {
            case Percent:
                m_sim_victor.set(outputValue);
                break;
            default:
                break;
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
        return m_sim_victor.getInverted() ? -m_sim_victor.get() : m_sim_victor.get();
    }

    //**************************************************************************
    public void setInverted(boolean invert)
    {
        m_sim_victor.setInverted(invert);
    }

    //**************************************************************************
    public void close()
    {
        m_sim_victor.close();
    }
}