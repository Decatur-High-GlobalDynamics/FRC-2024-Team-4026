package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.modules.leds.Color;
import frc.robot.subsystems.LedSubsystem;
import frc.lib.core.ILogSource;

public class FlashLightsOffCommand extends Command implements ILogSource
{
	public static LedSubsystem ledSubsystem;
	public static int currentRainbowColor;

	public FlashLightsOffCommand(LedSubsystem ledSubsystem)
	{
		this.ledSubsystem = ledSubsystem;
	}
	@Override
	public void initialize()
	{
		logFine("Flash Light Off Command Started");
	}
	@Override
	public void execute()
	{
		ledSubsystem.progress -= 0.02;
		currentRainbowColor++;
		ledSubsystem.setAllPixels(LedSubsystem.calcBlending(ledSubsystem.lastColor,
				Color.fromHSV(currentRainbowColor % 360, 255, 255), 1 - ledSubsystem.progress),
				false);

	}
	
	@Override
	public void end(boolean interrupted)
	{
		logFine("Flash Lights Off Command Finished");
	}
}
