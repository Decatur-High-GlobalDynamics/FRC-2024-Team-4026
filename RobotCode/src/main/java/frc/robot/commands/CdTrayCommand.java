package frc.robot.commands;

import java.time.LocalTime;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.core.util.Timer;
import frc.robot.subsystems.CdTraySubsystem;
import frc.lib.core.ILogSource;
public class CdTrayCommand extends Command implements ILogSource
{
	private final CdTraySubsystem CdTray;

	private Value cdMode;
	private Timer startTimer;
	private int timeToWait = 100;
	// milliseconds * 1000000

	private boolean closed;

	public CdTrayCommand(CdTraySubsystem CdTray)
	{
		logFiner("Constructing CDTray command...");
		this.CdTray = CdTray;
		this.closed = true;
		addRequirements(CdTray);
	}

	public void initialize()
	{
		logFine("CDtray Command Started...");
		CdTray.setSolenoid(cdMode);
		startTimer = new Timer(timeToWait);
	}

	@Override
	public void execute()
	{

		CdTray.closed = true;

		if (CdTray.getCdArmLeft().get() == Value.kOff && startTimer == null)
		{
			CdTray.setSolenoid(cdMode);
			startTimer = new Timer(timeToWait);
		}
	}

	@Override
	public boolean isFinished()
	{
		return startTimer != null && startTimer.isDone();
	}

	@Override
	public void end(boolean interrupted)
	{
		logFiner("CDtray Command Finished");
		CdTray.getCdArmLeft().set(Value.kOff);
	}

}
