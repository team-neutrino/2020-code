package frc.robot.subsystems;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;




public class DriveSubsystem extends SubsystemBase{

   private CANSparkMax leftMotor1 = new CANSparkMax(0, MotorType.kBrushless);
   private CANSparkMax leftMotor2 = new CANSparkMax(1, MotorType.kBrushless);
   private CANSparkMax rightMotor1 = new CANSparkMax(2, MotorType.kBrushless);
   private CANSparkMax rightMotor2 = new CANSparkMax(3, MotorType.kBrushless);

   private SpeedControllerGroup lSpeedControllerGroup = new SpeedControllerGroup(leftMotor1, leftMotor2);
   private SpeedControllerGroup rSpeedControllerGroup = new SpeedControllerGroup(rightMotor1, rightMotor2);
   private CANEncoder lEncoder = new CANEncoder(leftMotor1);
   private CANEncoder rEncoder = new CANEncoder(rightMotor1);


public DriveSubsystem(){

    lEncoder.setPositionConversionFactor(Constants.kDriveEncoderConversion);
    rEncoder.setPositionConversionFactor(Constants.kDriveEncoderConversion);
    

}

public void tankDrive(double leftPower, double rightPower){
    leftMotor1.set(leftPower);
    leftMotor2.set(leftPower);
    rightMotor1.set(-rightPower);
    rightMotor2.set(-rightPower);
}

}