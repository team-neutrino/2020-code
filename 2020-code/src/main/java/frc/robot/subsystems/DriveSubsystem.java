
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
    private CANEncoder m_lEncoder;
    private CANEncoder m_rEncoder;
    private AHRS m_navX = new AHRS(SPI.Port.kMXP);
    private final DifferentialDriveOdometry m_odometry;

    private double m_lrpm;
    private double m_rrpm;
    private double m_lrev;
    private double m_rrev;

    private double velocity = 0;
    public DriveSubsystem()
    {
        m_leftMotor1.restoreFactoryDefaults();
        m_leftMotor2.restoreFactoryDefaults();
        m_rightMotor1.restoreFactoryDefaults();
        m_rightMotor2.restoreFactoryDefaults();

        m_rightMotors.setInverted(true);
        m_leftMotors.setInverted(false);

        m_leftMotor1.setIdleMode(IdleMode.kBrake);
        m_leftMotor2.setIdleMode(IdleMode.kBrake);
        m_rightMotor1.setIdleMode(IdleMode.kBrake);
        m_rightMotor2.setIdleMode(IdleMode.kBrake);

        m_lEncoder = m_leftMotor1.getEncoder();
        m_rEncoder = m_rightMotor1.getEncoder();

        // position and velocity are in RPM of the drive wheels
        m_lEncoder.setPositionConversionFactor(DriveConstants.K_GEAR_RATIO);
        m_rEncoder.setPositionConversionFactor(DriveConstants.K_GEAR_RATIO);
        m_lEncoder.setVelocityConversionFactor(DriveConstants.K_GEAR_RATIO);
        m_rEncoder.setVelocityConversionFactor(DriveConstants.K_GEAR_RATIO);

        m_lEncoder.setPosition(0);
        m_rEncoder.setPosition(0);

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    }

    @Override
    public void periodic()
    {
        m_lrpm = m_lEncoder.getVelocity();
        m_rrpm = m_rEncoder.getVelocity();
        m_lrev = m_lEncoder.getPosition();
        m_rrev = m_rEncoder.getPosition();
        double lmeters = rev_to_m(m_lrev);
        double rmeters = rev_to_m(m_rrev);

        m_odometry.update(Rotation2d.fromDegrees(getHeading()), lmeters, rmeters);

        SmartDashboard.putNumber("Left RPM", m_lrpm);
        SmartDashboard.putNumber("Right RPM", m_rrpm);
        SmartDashboard.putNumber("Left m/s", rpm_to_mps(m_lrpm));
        SmartDashboard.putNumber("Right m/s", rpm_to_mps(m_rrpm));
        SmartDashboard.putNumber("Left m", lmeters);
        SmartDashboard.putNumber("Right m", rmeters);
        SmartDashboard.putNumber("NavX Yaw", m_navX.getYaw());
        SmartDashboard.putNumber("NavX Angle", m_navX.getAngle());
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
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds()
    {
        return new DifferentialDriveWheelSpeeds(rpm_to_mps(m_lrpm), rpm_to_mps(m_rrpm));
    }

    public double rpm_to_mps(double rpm)
    {
        final double METERS_PER_REV = DriveConstants.K_WHEEL_CIRCUMFERENCE;
        final double SEC_PER_MIN = 60;
        return rpm * METERS_PER_REV / SEC_PER_MIN;
    }

    public double rev_to_m(double rev)
    {
        final double METERS_PER_REV = DriveConstants.K_WHEEL_CIRCUMFERENCE;
        return rev * METERS_PER_REV;
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
