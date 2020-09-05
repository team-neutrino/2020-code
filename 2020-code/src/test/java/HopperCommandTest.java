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