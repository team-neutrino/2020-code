/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase
{
    private TalonSRX m_ClimbMotor = new TalonSRX(CanId.MOTOR_CONTROLLER_CLIMBER);

    /**
     * Creates a new ClimberSubsystem.
     */
    public ClimberSubsystem()
    {
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }

    public void elevatorUp()
    {
        m_ClimbMotor.set(ControlMode.PercentOutput, ClimberConstants.CLIMBER_MOTOR_POWER);
    }

    public void elevatorDown()
    {
        m_ClimbMotor.set(ControlMode.PercentOutput, -ClimberConstants.CLIMBER_MOTOR_POWER);
    }

    public void elevatorStop()
    {
        m_ClimbMotor.set(ControlMode.PercentOutput, 0);
    }
}
