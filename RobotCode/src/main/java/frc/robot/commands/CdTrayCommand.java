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
	// Milliseconds, courtesy of Harrison

	private boolean closed;

	public CdTrayCommand(CdTraySubsystem CdTray, Value cdMode)
	{
		System.out.println("Constructing CdTrayCommand...");
		this.CdTray = CdTray;
		this.closed = true;
		this.cdMode = cdMode;
		addRequirements(CdTray);
	}

	public void initialize()
	{
		System.out.println("Initializing CdTrayCommand...");
		CdTray.setSolenoid(cdMode);
		startTimer = new Timer(timeToWait);
	}
}

// no
// ):<