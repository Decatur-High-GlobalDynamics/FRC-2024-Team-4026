package frc.lib.modules.leds.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.modules.leds.LedSubsystem;
import frc.lib.modules.leds.TeamColor;

public class HoldLedCommand extends Command {

    private LedSubsystem led;
    private TeamColor desiredColor;
    private TeamColor lastColor;

    public HoldLedCommand(LedSubsystem led, TeamColor desiredColor) {
        this.led = led;
        this.desiredColor = desiredColor;

        addRequirements(led);
    }

    @Override
    public void initialize() {
        lastColor = new TeamColor(led.getCurrentColor().r, 
                led.getCurrentColor().g, 
                led.getCurrentColor().b);

        // Set LED strip to desired color
        led.setAllPixels(desiredColor);
    }

    @Override
    public void end(boolean interrupted) {
        // Revert LED strip to color before command started
        led.setAllPixels(lastColor);
    }
    
}
