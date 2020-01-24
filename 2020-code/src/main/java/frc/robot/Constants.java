/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;


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
        pooblic static final double KS_VOLTS = 0 ;
        public static final double KV_VOLT_SECONDS_PER_METER = 0;
        public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0;
        public static final double KP_DRIVE_VEL = 0;
        public static final double K_TRACK_WIDTH_METERS = 0;
        public static final DifferentialDriveKinematics K_DRIVE_KINEMATICS =
            new DifferentialDriveKinematics(K_TRACK_WIDTH_METERS);

        public static final double K_MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double K_RAMSETE_B = 2;
        public static final double K_RAMSETE_ZETA = 0.7; 
    
        private static final double K_GEAR_RATIO = 9.0/84.0;
        private static final double K_WHEEL_CIRCUMFERENCE = Math.PI*0.127;
        public static final double K_DRIVE_ENCODER_CONVERSION = K_GEAR_RATIO*K_WHEEL_CIRCUMFERENCE;

        public static final int MOTOR_CONTROLLER_DRIVER_LEFT1 = 1;
        public static final int MOTOR_CONTROLLER_DRIVER_LEFT2 = 2;
        public static final int MOTOR_CONTROLLER_DRIVER_RIGHT1 = 3;
        public static final int MOTOR_CONTROLLER_DRIVER_RIGHT2 = 4;
        public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = //this used to be type var, why?
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(DriveConstants.KS_VOLTS,
                                        DriveConstants.KV_VOLT_SECONDS_PER_METER,
                                        DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
                                        DriveConstants.K_DRIVE_KINEMATICS,
                                        10);
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_POWER = 1;
        public static final int MOTOR_CONTROLLER_INTAKE = 15;
    }

    public static final class ControllerPorts {
        public static final int XBOX_CONTROLLER_PORT = 2;
    }

    public static final class JoystickConstants {
        public static final int LEFT_JOYSTICK_PORT = 0;
        public static final int RIGHT_JOYSTICK__PORT = 1;
        public static final double DEADZONE_SIZE = 0.1;
        public static final double JOYSTICK_CURVE = 1.0;
    }

    public static final class ShooterConstants
    {
        //TODO get actual constants
        public static final int WHEEL_MOTOR_PORT = 14;
        public static final int WHEEL_ENCODER_PORT_1 = 0;
        public static final int WHEEL_ENCODER_PORT_2 = 1;
        public static final int WHEEL_ENCODER_DIST_PER_PULSE = 0;
        public static final double WHEEL_P = 0;
        public static final double WHEEL_I = 0;
        public static final double WHEEL_D = 0;
        public static final double KS_VOLTS = 0;
        public static final double KV_VOLT_SEC_PER_ROTATION = 0;
    }
}
