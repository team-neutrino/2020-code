/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants
    {
        //TODO get actual constants 
        public static final double ksVolts = 0 ;
        public static final double kvVoltSecondsPerMeter = 0;
        public static final double kaVoltSecondsSquaredPerMeter = 0;
        public static final double kPDriveVel = 0;
        public static final double kTrackwidthMeters = 0;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7; 
    
        private static final double kGearRatio = 9.0/84.0;
        private static final double kWheelCircumference = Math.PI*0.127;
        public static final double kDriveEncoderConversion = kGearRatio*kWheelCircumference;

        public static final int motorControllerDriveLeft1 = 1;
        public static final int motorControllerDriveLeft2 = 2;
        public static final int motorControllerDriveRight1 = 3;
        public static final int motorControllerDriveRight2 = 4;
    }

    public static final class IntakeConstants {
        public static final int intakeMotorPower = 1;
    }

    public static final class ControllerPorts {
        public static final int XBoxControllerPort = 0;
    }
}
