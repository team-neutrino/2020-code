package frc.robot.subsystems;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


public class DriveSubsystem extends SubsystemBase{

   private CANSparkMax m_leftMotor1 = new CANSparkMax(DriveConstants.MOTOR_CONTROLLER_DRIVER_LEFT1, MotorType.kBrushless);
   private CANSparkMax m_leftMotor2 = new CANSparkMax(DriveConstants.MOTOR_CONTROLLER_DRIVER_LEFT2, MotorType.kBrushless);
   private CANSparkMax m_rightMotor1 = new CANSparkMax(DriveConstants.MOTOR_CONTROLLER_DRIVER_RIGHT1, MotorType.kBrushless);
   private CANSparkMax m_rightMotor2 = new CANSparkMax(DriveConstants.MOTOR_CONTROLLER_DRIVER_RIGHT2, MotorType.kBrushless);

   private SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(m_leftMotor1, m_leftMotor2);
   private SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(m_rightMotor1, m_rightMotor2);
   private CANEncoder m_lEncoder = new CANEncoder(m_leftMotor1);
   private CANEncoder m_rEncoder = new CANEncoder(m_rightMotor1);
   private AHRS m_navX = new AHRS(SPI.Port.kMXP);
   private final DifferentialDriveOdometry m_odometry;

    public DriveSubsystem()
    {
        m_lEncoder.setPositionConversionFactor(DriveConstants.K_DRIVE_ENCODER_CONVERSION);
        m_rEncoder.setPositionConversionFactor(DriveConstants.K_DRIVE_ENCODER_CONVERSION); 
        m_lEncoder.setPosition(0);
        m_rEncoder.setPosition(0);
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    }

    @Override
    public void periodic()
    {
        m_odometry.update(Rotation2d.fromDegrees(getHeading()),m_lEncoder.getPosition(), m_rEncoder.getPosition()); 
    }

    public void tankDrive(double leftPower, double rightPower)
    {
        m_leftMotors.set(leftPower);
        m_rightMotors.set(-rightPower);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts)
    {
        m_leftMotors.setVoltage(leftVolts);
        m_rightMotors.setVoltage(-rightVolts);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds()
    {
        return new DifferentialDriveWheelSpeeds(m_lEncoder.getVelocity(), m_rEncoder.getVelocity());
    }

    //Returns robot angle in degrees from 180 to 180
    public double getHeading()
    {
        return m_navX.getFusedHeading() * -1;
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
}