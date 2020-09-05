package frc.robot.util.NeuMotor;

//******************************************************************************
public enum NeuMotorType
{
	None(0),
	TalonSRX(1),
	CANSparkMax(2);

	public final int value;

	//**************************************************************************
	NeuMotorType(int initValue)
	{
		this.value = initValue;
	}
};
