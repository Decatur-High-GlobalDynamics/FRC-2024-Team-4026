package frc.lib.modules.leds.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.modules.leds.LedSubsystem;
import frc.lib.modules.leds.TeamColor;

public class FlashLedCommand extends InstantCommand {

    private TeamColor desiredColor;
    private int numFlashes;
    private LedSubsystem led;

    public FlashLedCommand(LedSubsystem led, TeamColor desiredColor, int numFlashes) {
        this.led = led;
        this.desiredColor = desiredColor;
        this.numFlashes = numFlashes;

        addRequirements(led);
    }

    @Override
    public void initialize() {
        // Flash LED strip to desired color
        led.flashAllPixels(desiredColor, numFlashes);
    }
    
}
