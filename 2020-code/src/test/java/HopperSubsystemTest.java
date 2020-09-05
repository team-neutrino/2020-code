import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.*;

// wpilib robot
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// wpilib hal
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.sim.DriverStationSim;
import edu.wpi.first.wpilibj.DriverStation;

// junit
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

// robot code
import frc.robot.commands.HopperDefaultCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.util.NeuMotor;

//==============================================================================
// Hopper Subsystem Tests
//==============================================================================
public class HopperSubsystemTest
{
    public HopperSubsystem ss_hopper;
    public ShooterSubsystem ss_shooter;

    // public NeuMotor intake_motor;
    // public NeuMotor tower_motor;

    final double DELTA = 0.000001;

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
        CommandScheduler.getInstance().unregisterSubsystem(ss_hopper);
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

        when(ss_hopper.getTime()).thenReturn(0.0);
        when(ss_hopper.topBeamStatus()).thenReturn(true);
        when(ss_hopper.bottomBeamStatus()).thenReturn(true);
        when(ss_hopper.getPrevBotBeam()).thenReturn(true);
    }

    //==========================================================================
    // construct a real hopper subsystem
    protected void RealHopper()
    {
        // real hopper subsystem with mocked shooter
        ss_shooter = mock(ShooterSubsystem.class);
        ss_hopper = new HopperSubsystem(ss_shooter);
        CommandScheduler.getInstance().registerSubsystem(ss_hopper);
    }

    //==========================================================================
    private void SetHopperDefaultCommand()
    {
        HopperDefaultCommand dfltCommand = new HopperDefaultCommand(ss_hopper);
        CommandScheduler.getInstance().setDefaultCommand(ss_hopper, dfltCommand);
    }

    //==========================================================================
    private void Tick()
    {
        CommandScheduler.getInstance().run();

        boolean status = ss_hopper.bottomBeamStatus();
        when(ss_hopper.getPrevBotBeam()).thenReturn(status);
    }

    //==========================================================================
    // Subsystem Tests
    //==========================================================================

    //==========================================================================
    @Test
    public void RollerTowardsIntakeSetsPercentOut()
    {
        RealHopper();

        ss_hopper.rollerTowardsIntake();
        assertEquals(ss_hopper.GetIntakeMotorPercentOutput(), 0.3, DELTA);
    }

    //==========================================================================
    // periodic function sets the percent out on the hopper's intake motor
    @Test
    public void IntakeMotorAlwaysTurning()
    {
        RealHopper();

        // tick
        CommandScheduler.getInstance().run();

        // Verify that the intake motor is set towards intake
        assertEquals(ss_hopper.GetIntakeMotorPercentOutput(), 0.3, DELTA);
    }

    //==========================================================================
    @Test
    public void TowerMotorRunsAtSetSpeeds()
    {
        RealHopper();

        ss_hopper.towerIndexing();
        assertEquals(ss_hopper.GetTowerMotorPercentOutput(), 0.5, DELTA);

        ss_hopper.towerShoot();
        assertEquals(ss_hopper.GetTowerMotorPercentOutput(), 1.0, DELTA);

        ss_hopper.stop();
        assertEquals(ss_hopper.GetTowerMotorPercentOutput(), 0.0, DELTA);

        ss_hopper.reverse();
        assertEquals(ss_hopper.GetTowerMotorPercentOutput(), -0.5, DELTA);
    }

    //==========================================================================
    @Test
    public void TowerWontShootWhenShooterIsSlow()
    {
        RealHopper();

        // hopper is stopped
        ss_hopper.stop();
        CommandScheduler.getInstance().run();
        assertEquals(ss_hopper.GetTowerMotorPercentOutput(), 0.0, DELTA);

        // slow shooter: don't shoot
        when(ss_shooter.getVelocity()).thenReturn(200.0);
        ss_hopper.conditionalTowerShoot();
        assertEquals(ss_hopper.GetTowerMotorPercentOutput(), 0.0, DELTA);

        // fast shooter: do shoot
        when(ss_shooter.getVelocity()).thenReturn(70000.0);
        ss_hopper.conditionalTowerShoot();
        assertEquals(ss_hopper.GetTowerMotorPercentOutput(), 1.0, DELTA);

    }

    //==========================================================================
    // Command Tests
    //==========================================================================

    //==========================================================================
    // the default command calls some methods in the subsystem
    @Test
    public void HopperStopsWhenTopBeamBreakSensesBall()
    {
        MockHopper();
        SetHopperDefaultCommand();

        // init
        Tick();
        Tick();

        // covered top sensor stops feed
        when(ss_hopper.topBeamStatus()).thenReturn(false);
        Tick();
        // Hopper stops
        verify(ss_hopper, times(1)).stop();
    }

    //==========================================================================
    // the default command calls some methods in the subsystem
    @Test
    public void HopperFeedsWhenBottomBeamBreakSensesBall()
    {
        final int COVERED_DURATION = 15;
        MockHopper();
        SetHopperDefaultCommand();

        // init
        Tick();
        Tick();

        // Hopper indexes balls when bottom sensor is covered
        when(ss_hopper.bottomBeamStatus()).thenReturn(false);
        for (int ii = 0; ii < COVERED_DURATION; ii++)
        {
            Tick();
        }
        verify(ss_hopper, times(COVERED_DURATION)).towerIndexing();

        // transition to uncovered
        when(ss_hopper.bottomBeamStatus()).thenReturn(true);
        Tick();
        verify(ss_hopper, times(1)).startTimer();

        // runs for some duration and then stop tower motor
        when(ss_hopper.getTime()).thenReturn(0.0001);
        Tick();
        verify(ss_hopper, times(0)).stop();

        when(ss_hopper.getTime()).thenReturn(0.02);
        Tick();
        verify(ss_hopper, times(1)).stop();
    }
}
