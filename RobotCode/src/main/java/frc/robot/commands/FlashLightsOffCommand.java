package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.modules.leds.Color;
import frc.robot.subsystems.LedSubsystem;

public class FlashLightsOffCommand extends Command
{
	public static LedSubsystem ledSubsystem;
	public static int currentRainbowColor;

	public FlashLightsOffCommand(LedSubsystem ledSubsystem)
	{
		this.ledSubsystem = ledSubsystem;
	}

	public void execute()
	{
		ledSubsystem.progress -= 0.02;
		currentRainbowColor++;
		ledSubsystem.setAllPixels(LedSubsystem.calcBlending(ledSubsystem.lastColor,
				Color.fromHSV(currentRainbowColor % 360, 255, 255), 1 - ledSubsystem.progress),
				false);

	}

	public void end(boolean interrupted)
	{

	}
}
