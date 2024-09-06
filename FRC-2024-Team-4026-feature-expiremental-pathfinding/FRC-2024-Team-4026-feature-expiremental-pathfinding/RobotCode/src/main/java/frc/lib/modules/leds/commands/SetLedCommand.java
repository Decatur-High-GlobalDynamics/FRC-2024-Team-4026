package frc.lib.modules.leds.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.modules.leds.LedSubsystem;
import frc.lib.modules.leds.TeamColor;

public class SetLedCommand extends InstantCommand {

    private TeamColor desiredColor;
    private LedSubsystem led;

    public SetLedCommand(LedSubsystem led, TeamColor desiredColor) {
        this.led = led;
        this.desiredColor = desiredColor;

        addRequirements(led);
    }

    @Override
    public void initialize() {
        // Set LED strip to desired color
        led.setAllPixels(desiredColor);
    }
    
}
