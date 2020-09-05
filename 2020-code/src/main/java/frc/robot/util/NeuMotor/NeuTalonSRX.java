
package frc.robot.util.NeuMotor;

// CAN Talon SRX
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

//******************************************************************************
class NeuTalonSRX implements NeuMotorIf
{
    private TalonSRX m_TalonSRX;

    //**************************************************************************
    public NeuTalonSRX(int deviceNumber)
    {
        m_TalonSRX = new TalonSRX(deviceNumber);
    }

    //**************************************************************************
    public void set(NeuMotorMode mode, double outputValue)
    {
        ControlMode talon_control_mode;
        switch (mode)
        {
            case Percent:
                talon_control_mode = ControlMode.PercentOutput;
                break;
            case Position:
                talon_control_mode = ControlMode.Position;
                break;
            case Velocity:
                talon_control_mode = ControlMode.Velocity;
                break;
            case Current:
                talon_control_mode = ControlMode.Current;
                break;
            case Follower:
                talon_control_mode = ControlMode.Follower;
                break;
            default:
                talon_control_mode = ControlMode.Disabled;
                break;
        }
        set(talon_control_mode, outputValue);
    }

    //**************************************************************************
    public void set(double outputValue)
    {
        set(NeuMotorMode.Percent, outputValue);
    }

    //**************************************************************************
    // call TalonSRX set() api
    private void set(ControlMode mode, double outputValue)
    {
        m_TalonSRX.set(mode, outputValue);
    }

    //**************************************************************************
    public double get()
    {
        return m_TalonSRX.getMotorOutputPercent();
    }

    //**************************************************************************
    public void setInverted(boolean invert)
    {
        m_TalonSRX.setInverted(invert);
    }

    //**************************************************************************
    public void close()
    {
    }
}