
package frc.robot.subsystems;

import static org.mockito.Mockito.*;

// wpilib robot
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;

// wpilib hal
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.sim.DriverStationSim;
import edu.wpi.first.wpilibj.DriverStation;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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
    HopperSubsystem mockedHopperSubsystem;

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

        // mocked subsystem
        mockedHopperSubsystem = mock(HopperSubsystem.class);
        reset(mockedHopperSubsystem);
        CommandScheduler.getInstance().registerSubsystem(mockedHopperSubsystem);
    }

    //==========================================================================
    // periodic is called in command loop
    @Test
    public void HopperSubsystemCallsPeriodic()
    {

        ShooterSubsystem shooter_mock =  mock(ShooterSubsystem.class);
        HopperSubsystem hopper = new HopperSubsystem( shooter_mock );
        TalonSRX motor = mock(TalonSRX.class);
        mockedHopperSubsystem.SetIntakeMotor( motor );

        CommandScheduler.getInstance().registerSubsystem(hopper);

        CommandScheduler.getInstance().run();

        // Verify that periodic was called once
        // verify(hopper, times(1)).periodic();

        System.out.println(hopper.getRollerMotorSetpoint() );
        assert(hopper.getRollerMotorSetpoint()  != 0);
    }

    //==========================================================================
    // the default command calls some methods in the subsystem
    @Test
    public void HopperDefaultCommandChecksSensorStates()
    {
        // set default command
        HopperDefaultCommand dfltCommand = new HopperDefaultCommand(mockedHopperSubsystem);
        CommandScheduler.getInstance().setDefaultCommand(mockedHopperSubsystem, dfltCommand);

        // runs command init
        CommandScheduler.getInstance().run();
        // runs command execute
        CommandScheduler.getInstance().run();

        // Verify that methods in the subsysteem were called
        verify(mockedHopperSubsystem, times(1)).bottomBeamStatus();
        verify(mockedHopperSubsystem, times(1)).topBeamStatus();
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

}
