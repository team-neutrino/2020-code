/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
/*import edu.wpi.first.wpilibj.Encoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;*/
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

/**
 * Add your docs here.
 */
public class IntakeSubsystem {
    private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.MOTOR_CONTROLLER_INTAKE, MotorType.kBrushless);
    private DoubleSolenoid pusher = new DoubleSolenoid(0, 1);

    public void setIntake(boolean on) {
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
}
