package frc.robot.subsystems;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.getMotorTempearture;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class DriveSubsystem extends SubsystemBase{

   private CANSparkMax leftMotor1 = new CANSparkMax(DriveConstants.MOTOR_CONTROLLER_DRIVER_LEFT1, MotorType.kBrushless);
   private CANSparkMax leftMotor2 = new CANSparkMax(DriveConstants.MOTOR_CONTROLLER_DRIVER_LEFT2, MotorType.kBrushless);
   private CANSparkMax rightMotor1 = new CANSparkMax(DriveConstants.MOTOR_CONTROLLER_DRIVER_RIGHT1, MotorType.kBrushless);
   private CANSparkMax rightMotor2 = new CANSparkMax(DriveConstants.MOTOR_CONTROLLER_DRIVER_RIGHT2, MotorType.kBrushless);
   private SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(leftMotor1, leftMotor2);
   private SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(rightMotor1, rightMotor2);
   private CANEncoder lEncoder = new CANEncoder(leftMotor1);
   private CANEncoder rEncoder = new CANEncoder(rightMotor1);

   private AHRS navX = new AHRS(SPI.Port.kMXP);
   private PowerDistributionPanel PDP = new PowerDistributionPanel();
   private final DifferentialDriveOdometry m_odometry;

    public DriveSubsystem()
    {
        lEncoder.setPositionConversionFactor(DriveConstants.K_DRIVE_ENCODER_CONVERSION);
        rEncoder.setPositionConversionFactor(DriveConstants.K_DRIVE_ENCODER_CONVERSION); 
        lEncoder.setPosition(0);
        rEncoder.setPosition(0);
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    }

    @Override
    public void periodic()
    {
        m_odometry.update(Rotation2d.fromDegrees(getHeading()),lEncoder.getPosition(), rEncoder.getPosition()); 
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
        return new DifferentialDriveWheelSpeeds(lEncoder.getVelocity(), rEncoder.getVelocity());
    }

    //Returns robot angle in degrees from 180 to 180
    public double getHeading()
    {
        return 0; //TODO implement this method
    }

    public Pose2d getPose()
    {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose)
    {
        lEncoder.setPosition(0);
        rEncoder.setPosition(0);
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public void getPDPCurrent() {
        double currentLeftOne = PDP.getCurrent(DriveConstants.MOTOR_CONTROLLER_DRIVER_LEFT1);
        double currentLeftTwo = PDP.getCurrent(DriveConstants.MOTOR_CONTROLLER_DRIVER_LEFT2);
        double currentRightOne = PDP.getCurrent(DriveConstants.MOTOR_CONTROLLER_DRIVER_RIGHT1);
        double currentRightTwo = PDP.getCurrent(DriveConstants.MOTOR_CONTROLLER_DRIVER_RIGHT2);
        ArrayList<Double> currents = new ArrayList<Double>();
        currents.add(currentLeftOne);
        currents.add(currentLeftTwo);
        currents.add(currentRightOne);
        currents.add(currentRightTwo);
        System.out.println("MOTOR_CONTROLLER_DRIVER_LEFT1: " + currents.get(0));
        System.out.println("MOTOR_CONTROLLER_DRIVER_LEFT2: " + currents.get(1));
        System.out.println("MOTOR_CONTROLLER_DRIVER_RIGHT1: " + currents.get(2));
        System.out.println("MOTOR_CONTROLLER_DRIVER_RIGHT2: " + currents.get(3));

    }

    /*
    public ArrayList<Double> getCANTemp() {
        double tempLeftOne = CANSparkMax.getMotorTempearture(DriveConstants.MOTOR_CONTROLLER_DRIVER_LEFT1);
    }
    */
}