/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanId;
import frc.robot.Constants.IntakeConstants;

/**
 * Add your docs here.
 */
public class IntakeSubsystem extends SubsystemBase 
{
    private AnalogPotentiometer adjustMotorPotentiometer = new AnalogPotentiometer(Constants.IntakeConstants.ADJUST_MOTOR_ENCODER);
    private TalonSRX intakeMotor = new TalonSRX(CanId.MOTOR_CONTROLLER_INTAKE);
    private TalonSRX intakeAdjustMotor = new TalonSRX(CanId.MOTOR_CONTROLLER_INTAKE_ADJUST);
    private PowerDistributionPanel PDP = new PowerDistributionPanel();

    private PIDController PID = new PIDController(Constants.PIDConstants.PROPORTION_COEFFICIENT, 
                                                Constants.PIDConstants.INTEGRAL_COEFFICIENT, 
                                                Constants.PIDConstants.DERIVATIVE_COEFFICEINT);

    public void setIntake(boolean on)   
    {
        if (on == true) 
        {
            intakeMotor.set(ControlMode.PercentOutput, IntakeConstants.INTAKE_MOTOR_POWER);
        }
        else
        {
            intakeMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    public void setIntakeAngle(double angle)
    {
        PID.setSetpoint(angle);
    }

    public double getIntakeSetpoint()
    {
        return PID.getSetpoint();
    }

    public double getPotentiometerReading()
    {
        return adjustMotorPotentiometer.get();
    }
  
    public void getPDPCurrent() 
    {
        double currentIntakeMotor = PDP.getCurrent(CanId.MOTOR_CONTROLLER_INTAKE);
        System.out.println("MOTOR_CONTROLLER_INTAKE: " + currentIntakeMotor);
    }

    public void intakePrinter()
    {
        SmartDashboard.putNumber("Arm setpoint: ", PID.getSetpoint());
        SmartDashboard.putNumber("Potentiometer reading: ", getPotentiometerReading());
    }

}
