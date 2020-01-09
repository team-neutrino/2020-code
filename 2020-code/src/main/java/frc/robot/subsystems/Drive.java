package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;




public class Drive extends SubsystemBase{

   private CANSparkMax leftMotor1 = new CANSparkMax(0, MotorType.kBrushless);
   private CANSparkMax leftMotor2 = new CANSparkMax(1, MotorType.kBrushless);
   private CANSparkMax rightMotor1 = new CANSparkMax(2, MotorType.kBrushless);
   private CANSparkMax rightMotor2 = new CANSparkMax(3, MotorType.kBrushless);

public Drive(){

}

public void tankDrive(double leftPower, double rightPower){
    leftMotor1.set(leftPower);
    leftMotor2.set(leftPower);
    rightMotor1.set(-rightPower);
    rightMotor2.set(-rightPower);
}

}