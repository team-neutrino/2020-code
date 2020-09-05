
package frc.robot.util.NeuMotor;

import frc.robot.util.NeuMotor.NeuMotorMode;

//
// Note: if a method's control mode is not specified, it will be assumed to be % output.
//

//******************************************************************************
interface NeuMotorIf
{
    // set the desired motor controller output
    public void set(NeuMotorMode mode, double outputValue);

    public void set(double outputValue);

    // get the desired motor controller output
    public double get();

    // configuration
    public void setInverted(boolean invert);

    // some speed controller objects need to be destructed
    public void close();
}