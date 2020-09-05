import static org.mockito.Mockito.*;

// wpilib robot
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// junit
import org.junit.Test;

// robot code
import frc.robot.commands.HopperDefaultCommand;

//==============================================================================
// Hopper Subsystem Commands Tests
//==============================================================================
public class HopperCommandTest extends HopperSubsystemTest
{
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
        for( int ii = 0; ii < COVERED_DURATION; ii++)
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