/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase
{
    private TalonSRX m_ClimbElevator = new TalonSRX(CanId.MOTOR_CONTROLLER_CLIMBER);
    //private CANSparkMax m_ClimbWinch = new CANSparkMax(CanId.MOTOR_CONTROLLER_CLIMBERWINCH, MotorType.kBrushless);

    /**
     * Creates a new ClimberSubsystem.
     */
    public ClimberSubsystem()
    {
        m_ClimbElevator.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }

    public void elevatorUp()
    {
        m_ClimbElevator.set(ControlMode.PercentOutput, ClimberConstants.CLIMBER_MOTOR_POWER);
    }

    public void elevatorDown()
    {
        m_ClimbElevator.set(ControlMode.PercentOutput, -ClimberConstants.CLIMBER_MOTOR_POWER);
    }

    public void winchClimb()
    {
        //m_ClimbWinch.set(ClimberConstants.CLIMBER_MOTOR_WINCHPOWER);
    }

    public void elevatorStop()
    {
        m_ClimbElevator.set(ControlMode.PercentOutput, 0);
    }

    public void winchStop()
    {
        //m_ClimbWinch.set(0);
    }

    public void winchReverse()
    {
        //m_ClimbWinch.set(-ClimberConstants.CLIMBER_MOTOR_WINCHPOWER);
    }
}
