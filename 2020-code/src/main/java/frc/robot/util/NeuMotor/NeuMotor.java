
package frc.robot.util.NeuMotor;

// Neu motors
import frc.robot.util.NeuMotor.NeuMotorIf;
import frc.robot.util.NeuMotor.NeuMotorType;
import frc.robot.util.NeuMotor.NeuMotorMode;

// Motor controller wrappers
import frc.robot.util.NeuMotor.NeuTalonSRX;
import frc.robot.util.NeuMotor.NeuCANSparkMax;
import frc.robot.util.NeuMotor.NeuSimMotorController;

// for simulation support
import frc.robot.Robot;

//******************************************************************************
// This is a generic motor implementaion and makes the underlying motor
// controller class invisible.
//******************************************************************************
public class NeuMotor implements NeuMotorIf
{
    private NeuMotorIf m_motor_controller;
    static boolean m_sim = Robot.isSimulation();

    //**************************************************************************
    // motor controller factory
    public NeuMotor(NeuMotorType type, int device)
    {
        if (m_sim)
        {
            m_motor_controller = new NeuSimMotorController(device);
            return;
        }

        switch (type)
        {
            case TalonSRX:
                m_motor_controller = new NeuTalonSRX(device);
                break;
            case CANSparkMax:
                m_motor_controller = new NeuCANSparkMax(device);
                break;
            default:
                break;
        }
    }

    //**************************************************************************
    public void set(NeuMotorMode mode, double outputValue)
    {
        m_motor_controller.set(mode, outputValue);
    }

    //**************************************************************************
    public void set(double outputValue)
    {
        m_motor_controller.set(outputValue);
    }

    //**************************************************************************
    public double get()
    {
        return m_motor_controller.get();
    }

    //**************************************************************************
    public void setInverted(boolean invert)
    {
        m_motor_controller.setInverted(invert);
    }

    //**************************************************************************
    public void close()
    {
        m_motor_controller.close();
    }

}