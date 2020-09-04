
package frc.robot.subsystems;

import static org.mockito.Mockito.*;

// wpilib robot
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;
// import edu.wpi.first.wpilibj.PWMTalonSRX;

// wpilib hal
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.sim.DriverStationSim;
import edu.wpi.first.wpilibj.DriverStation;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

// junit
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

// robot code
import frc.robot.commands.HopperDefaultCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

//==============================================================================
public class HopperSubsystemTest
{
    HopperSubsystem ss_hopper;
    TalonSRX intake_motor;

    //==========================================================================
    @Before
    public void before()
    {
        // initialize the command scheduler
        CommandScheduler.getInstance().enable();
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearButtons();

        // mocked hardware
        HAL.initialize(500, 0);
        DriverStationSim dsSim = new DriverStationSim();
        dsSim.setDsAttached(true);
        dsSim.setAutonomous(false);
        dsSim.setEnabled(true);
        dsSim.setTest(true);
    }

    //==========================================================================
    // This is called after tests, and makes sure that nothing is left open and
    // everything is ready for the next test class
    @After
    public void after()
    {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearButtons();

        CommandScheduler.getInstance().disable();

        // deinit hardware
        DriverStation.getInstance().release();
        HAL.releaseDSMutex();
    }

    //==========================================================================
    // construct a mocked hopper subsystem
    private void MockHopper()
    {
        // mocked subsystem
        ss_hopper = mock(HopperSubsystem.class);
        reset(ss_hopper);
        CommandScheduler.getInstance().registerSubsystem(ss_hopper);
    }

    //==========================================================================
    // construct a real hopper subsystem
    private void RealHopper()
    {
        // real subsystem
        ShooterSubsystem shooter_mock = mock(ShooterSubsystem.class);
        ss_hopper = new HopperSubsystem( shooter_mock );
        CommandScheduler.getInstance().registerSubsystem(ss_hopper);
    } 

    //==========================================================================
    // construct a real hopper subsystem
    private void MockIntakeMotor()
    {
        // mock motor controller with real subsystem
        intake_motor = mock(TalonSRX.class);
        ss_hopper.SetIntakeMotor( intake_motor );
    } 

    //==========================================================================
    // TESTS
    //==========================================================================

    //==========================================================================
    // periodic is called in command loop
    @Test
    public void HopperSubsystemCallsPeriodic()
    {
        MockHopper();
        CommandScheduler.getInstance().run();

         // Verify that periodic was called once
        verify(ss_hopper, times(1)).periodic();
    }

    //==========================================================================
    // periodic function sets the percent out on the hopper's intake motor
    @Test
    public void RollerMotorAlwaysTurning()
    {
        RealHopper();
        MockIntakeMotor();

        CommandScheduler.getInstance().run();

        // Verify that motor is set towards intake
        verify( intake_motor, times(1) ).set( ControlMode.PercentOutput, 0.3 );
    }

    //==========================================================================
    // the default command calls some methods in the subsystem
    @Test
    public void HopperDefaultCommandChecksSensorStates()
    {
        MockHopper();
        // set default command
        HopperDefaultCommand dfltCommand = new HopperDefaultCommand(ss_hopper);
        CommandScheduler.getInstance().setDefaultCommand(ss_hopper, dfltCommand);

        // runs command init
        CommandScheduler.getInstance().run();
        // runs command execute
        CommandScheduler.getInstance().run();

        // Verify that methods in the subsysteem were called
        verify(ss_hopper, times(1)).bottomBeamStatus();
        verify(ss_hopper, times(1)).topBeamStatus();
    }


}
