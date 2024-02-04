package frc.robot.commands;

import java.time.LocalTime;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.core.util.Timer;
import frc.robot.subsystems.CdTraySubsystem;

public class CdTrayCommand extends Command
{
	private final CdTraySubsystem CdTray;

	private Value cdMode;
	private Timer startTimer;
	private int timeToWait = 100;
	// milliseconds * 1000000

	private boolean closed;

	public CdTrayCommand(CdTraySubsystem CdTray)
	{
		System.out.println("Constructing CdTrayCommand...");
		this.CdTray = CdTray;
		this.closed = true;
		addRequirements(CdTray);
	}

	public void initialize()
	{
		System.out.println("Initializing CdTrayCommand...");
		CdTray.setSolenoid(cdMode);
		startTimer = new Timer(timeToWait);
	}

	public void execute()
	{

		CdTray.closed = true;

		if (CdTray.getCdArmLeft().get() == Value.kOff && startTimer == null)
		{
			CdTray.setSolenoid(cdMode);
			startTimer = new Timer(timeToWait);
		}
	}

	public boolean isFinished()
	{
		return startTimer != null && startTimer.isDone();
	}

	public void end()
	{
		System.out.println("Ending CdTrayCommand...");
		CdTray.getCdArmLeft().set(Value.kOff);
	}

}
