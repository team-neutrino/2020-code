package frc.robot.util.NeuMotor;

//******************************************************************************
public enum NeuMotorMode
{
	Percent(0),
	Position(1),
	Velocity(2),
	Current(3),
	Follower(5),
	MotionProfile(6),
	MotionMagic(7),
	MotionProfileArc(10),
	Disabled(255);

	public final int value;


	//**************************************************************************
	NeuMotorMode(int initValue)
	{
		this.value = initValue;
	}
};
