
package frc.robot.subsystems;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanId;
import frc.robot.Constants.DriveConstants;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class DriveSubsystem extends SubsystemBase
{
    //private PowerDistributionPanel PDP = new PowerDistributionPanel();
    private CANSparkMax m_leftMotor1 = new CANSparkMax(CanId.MOTOR_CONTROLLER_DRIVER_LEFT1, MotorType.kBrushless);
    private CANSparkMax m_leftMotor2 = new CANSparkMax(CanId.MOTOR_CONTROLLER_DRIVER_LEFT2, MotorType.kBrushless);
    private CANSparkMax m_rightMotor1 = new CANSparkMax(CanId.MOTOR_CONTROLLER_DRIVER_RIGHT1, MotorType.kBrushless);
    private CANSparkMax m_rightMotor2 = new CANSparkMax(CanId.MOTOR_CONTROLLER_DRIVER_RIGHT2, MotorType.kBrushless);

    private SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(m_leftMotor1, m_leftMotor2);
    private SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(m_rightMotor1, m_rightMotor2);
    private CANEncoder m_lEncoder = new CANEncoder(m_leftMotor1);
    private CANEncoder m_rEncoder = new CANEncoder(m_rightMotor1);
    private AHRS m_navX = new AHRS(SPI.Port.kMXP);
    private final DifferentialDriveOdometry m_odometry;

    private double velocity = 0;
    public DriveSubsystem()
    {
        m_lEncoder.setPositionConversionFactor(-1 * DriveConstants.K_DRIVE_ENCODER_CONVERSION);
        m_rEncoder.setPositionConversionFactor(DriveConstants.K_DRIVE_ENCODER_CONVERSION);
        m_lEncoder.setVelocityConversionFactor(-1 * DriveConstants.K_DRIVE_ENCODER_CONVERSION);
        m_lEncoder.setVelocityConversionFactor(DriveConstants.K_DRIVE_ENCODER_CONVERSION);
        m_lEncoder.setPosition(0);
        m_rEncoder.setPosition(0);
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        m_rightMotors.setInverted(true);
        m_leftMotors.setInverted(false);
        m_leftMotor1.setIdleMode(IdleMode.kBrake);
        m_leftMotor2.setIdleMode(IdleMode.kBrake);
        m_rightMotor1.setIdleMode(IdleMode.kBrake);
        m_rightMotor2.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic()
    {
        m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_lEncoder.getPosition(), m_rEncoder.getPosition());
        //SmartDashboard.putData("Pose", getPose());
        SmartDashboard.putNumber("Left Velocity", -1 * m_lEncoder.getVelocity());
        SmartDashboard.putNumber("Right Velocity", m_rEncoder.getVelocity());
        SmartDashboard.putNumber("NavX Yaw", m_navX.getYaw());
        SmartDashboard.putNumber("NavX Angle", m_navX.getAngle());
        SmartDashboard.putNumber("Velocity", (m_lEncoder.getVelocity() * DriveConstants.K_DRIVE_ENCODER_CONVERSION));
        SmartDashboard.putNumber("Acceleration", getMaxAcceleration());
    }

    public void tankDrive(double leftPower, double rightPower)
    {
        m_leftMotors.set(leftPower);
        m_rightMotors.set(rightPower);

    }

    public void tankDriveVolts(double leftVolts, double rightVolts)
    {
        m_leftMotors.setVoltage(leftVolts);
        m_rightMotors.setVoltage(rightVolts);
        // System.out.println("Left Volts " + leftVolts);
        // System.out.println("Right Volts " + rightVolts);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds()
    {
        return new DifferentialDriveWheelSpeeds(m_lEncoder.getVelocity(), m_rEncoder.getVelocity());
    }

    //Returns robot angle in degrees from -180 to 180
    public double getHeading()
    {
        return m_navX.getYaw() * -1;
    }

    public Pose2d getPose()
    {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose)
    {
        m_lEncoder.setPosition(0);
        m_rEncoder.setPosition(0);
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public void getPDPCurrent()
    {
        //double currentLeftOne = PDP.getCurrent(CanId.MOTOR_CONTROLLER_DRIVER_LEFT1);
        //double currentLeftTwo = PDP.getCurrent(CanId.MOTOR_CONTROLLER_DRIVER_LEFT2);
        //double currentRightOne = PDP.getCurrent(CanId.MOTOR_CONTROLLER_DRIVER_RIGHT1);
        //double currentRightTwo = PDP.getCurrent(CanId.MOTOR_CONTROLLER_DRIVER_RIGHT2);
        // ArrayList<Double> currents = new ArrayList<Double>();
        //currents.add(currentLeftOne);
        //currents.add(currentLeftTwo);
        //currents.add(currentRightOne);
        //currents.add(currentRightTwo);
        // System.out.println("MOTOR_CONTROLLER_DRIVER_LEFT1: " + currents.get(0));
        // System.out.println("MOTOR_CONTROLLER_DRIVER_LEFT2: " + currents.get(1));
        // System.out.println("MOTOR_CONTROLLER_DRIVER_RIGHT1: " + currents.get(2));
        // System.out.println("MOTOR_CONTROLLER_DRIVER_RIGHT2: " + currents.get(3));

    }

    public double getMaxAcceleration()
    {
        double oldVelocity = velocity;
        velocity = m_lEncoder.getVelocity() * Constants.DriveConstants.K_DRIVE_ENCODER_CONVERSION;

        return (velocity - oldVelocity) / 2;
    }
    /*
     * public ArrayList<Double> getCANTemp() { double tempLeftOne =
     * CANSparkMax.getMotorTempearture(DriveConstants.MOTOR_CONTROLLER_DRIVER_LEFT1); }
     */
}
