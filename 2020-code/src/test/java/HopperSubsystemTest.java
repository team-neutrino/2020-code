import static org.mockito.Mockito.*;

// wpilib robot
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

//==============================================================================
// Hopper Subsystem Tests
//==============================================================================
public class HopperSubsystemTest
{
    public HopperSubsystem ss_hopper;
    public ShooterSubsystem ss_shooter;

    public TalonSRX intake_motor;
    public TalonSRX tower_motor;

    //==========================================================================
    @Before
    public void before()
    {
        // initialize the command scheduler
        CommandScheduler.getInstance().enable();
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearButtons();

        // init hardware
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
        ss_hopper.close();

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearButtons();

        CommandScheduler.getInstance().disable();

        // deinit hardware
        DriverStation.getInstance().release();
        HAL.releaseDSMutex();
    }

    //==========================================================================
    // construct a mocked hopper subsystem
    protected void MockHopper()
    {
        // mocked hopper subsystem
        ss_hopper = mock(HopperSubsystem.class);
        reset(ss_hopper);
        CommandScheduler.getInstance().registerSubsystem(ss_hopper);
    }

    //==========================================================================
    // construct a real hopper subsystem
    protected void RealHopper()
    {
        // real hopper subsystem with mocked shooter
        ss_shooter = mock( ShooterSubsystem.class );
        ss_hopper = new HopperSubsystem( ss_shooter );
        CommandScheduler.getInstance().registerSubsystem( ss_hopper );
    } 

    //==========================================================================
    // construct a mock intake motor
    protected void MockIntakeMotor()
    {
        intake_motor = mock(TalonSRX.class);
        ss_hopper.SetIntakeMotor( intake_motor );
    } 

    //==========================================================================
    // construct a mock tower motor
    protected void MockTowerMotor()
    {
        tower_motor = mock(TalonSRX.class);
        ss_hopper.SetTowerMotor( tower_motor );
    } 

    //==========================================================================
    // Subsystem Tests
    //==========================================================================

    //==========================================================================
    // periodic function sets the percent out on the hopper's intake motor
    @Test
    public void IntakeMotorAlwaysTurning()
    {
        RealHopper();
        MockIntakeMotor();

        CommandScheduler.getInstance().run();

        // Verify that the intake motor is set towards intake
        verify( intake_motor, times(1) ).set( ControlMode.PercentOutput, 0.3 );
    }

    //==========================================================================
    // calling stop() stops the tower motor
    @Test
    public void CallingStopWillStopTowerMotor()
    {
        RealHopper();
        MockTowerMotor();

        // CommandScheduler.getInstance().run();
        ss_hopper.stop();

        // Verify that the tower motor stops
        verify( tower_motor, times(1) ).set( ControlMode.PercentOutput, 0.0 );
    }
}
