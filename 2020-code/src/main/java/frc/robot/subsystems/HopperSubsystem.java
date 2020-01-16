package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase 
    { 
    private CANSparkMax hopperMotor = new CANSparkMax(HopperConstants.MOTOR_CONTROLLER_HOPPER, MotorType.kBrushed); 
    public HopperSubsystem() 
    {

    }
    public void RunHooper(boolean on)
    {
        if (on == true){
            hopperMotor.set(HopperConstants.HOPPER_MOTOR_POWER);
        }
        else{
            hopperMotor.set(0);
        }
    }
}
