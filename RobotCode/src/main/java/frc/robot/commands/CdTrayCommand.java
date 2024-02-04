package frc.robot.commands;

import java.time.LocalTime;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CdTraySubsystem;
import frc.lib.core.ILogSource;
public class CdTrayCommand extends Command implements ILogSource
{
	private final CdTraySubsystem CdTray;

	private Value cdMode;
	private LocalTime startTime;
	private long timeToWait = 100 * 1000000;
	// milliseconds * 1000000

	private boolean closed;

	public CdTrayCommand(CdTraySubsystem CdTray)
	{
		logFiner("Constructing CDTray command...");;
		this.CdTray = CdTray;
		this.closed = true;
		addRequirements(CdTray);
	}

	public void initialize()
	{
		logFine("Command Started");
		CdTray.setSolenoid(cdMode);
		startTime = LocalTime.now();
	}

	@Override
	public void execute()
	{

		CdTray.closed = true;

		if (CdTray.getCdArmLeft().get() == Value.kOff && startTime == null)
		{
			CdTray.setSolenoid(cdMode);
			startTime = LocalTime.now();
		}
	}

	@Override
	public boolean isFinished()
	{
		return startTime != null && LocalTime.now().minusNanos(timeToWait).compareTo(startTime) > 0;
	}

	@Override
	public void end(boolean interrupted)
	{
		logFine("Command Finished");
		CdTray.getCdArmLeft().set(Value.kOff);
	}

}
