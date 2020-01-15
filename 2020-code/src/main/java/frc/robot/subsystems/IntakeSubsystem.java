/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import java.util.ArrayList;
import frc.robot.Constants.IntakeConstants;

/**
 * Add your docs here.
 */
public class IntakeSubsystem
{
    private CANSparkMax intakeMotor = 
        new CANSparkMax(IntakeConstants.MOTOR_CONTROLLER_INTAKE, MotorType.kBrushless);
    private PowerDistributionPanel PDP = new PowerDistributionPanel();
    private DoubleSolenoid pusher = new DoubleSolenoid(0, 1);

    public void setIntake(boolean on)
    {
        if (on == true) 
            intakeMotor.set(IntakeConstants.INTAKE_MOTOR_POWER);
        else 
            intakeMotor.set(0);
    }

    public void setPusherIn()
    {
        pusher.set(DoubleSolenoid.Value.kReverse);
    }

    public void setPusherOut()
    {
        pusher.set(DoubleSolenoid.Value.kForward);
    }
  
    public void getPDPCurrent()
    {
        double currentIntakeMotor = PDP.getCurrent(IntakeConstants.MOTOR_CONTROLLER_INTAKE);
        System.out.println("MOTOR_CONTROLLER_INTAKE: " + currentIntakeMotor);
    }
}
