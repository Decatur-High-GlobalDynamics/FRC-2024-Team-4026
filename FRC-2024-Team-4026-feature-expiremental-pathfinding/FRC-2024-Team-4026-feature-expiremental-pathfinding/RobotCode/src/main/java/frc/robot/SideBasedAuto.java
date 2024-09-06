package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.core.Autonomous;

public class SideBasedAuto extends Autonomous {

    private enum Side {
        Amp("Amp Side"), Center("Center"), Source("Source Side");

        private final String autoName;

        private Side(String autoName) {
            this.autoName = autoName;
        }
    }

    private enum AutoMode {
        OnlyShoot("Only Shoot"),
        DriveBackwards("Drive Backwards"), OneNote("1 Note"), TwoNote("2 Note"),
        ThreeNote("3 Note"), FourNote("4 Note"), SubwooferDisrupt("Subwoofer Disrupt"),
        SourceDisrupt("Source Disrupt");

        private final String autoName;

        private AutoMode(String autoName) {
            this.autoName = autoName;
        }
    }

	private enum AutoDirection {
		SourceDirection("Source Direction"), AmpDirection("Amp Direction");

		private final String autoName;

		private AutoDirection(String autoName) {
            this.autoName = autoName;
        }
	}

    private final SendableChooser<Side> SideChooser;
    private final SendableChooser<AutoMode> AutoModeChooser;
	private final SendableChooser<AutoDirection> AutoDirectionChooser;

    public SideBasedAuto(frc.robot.RobotContainer robotContainer) {
        super(robotContainer);

        SideChooser = new SendableChooser<>();
        SideChooser.setDefaultOption(Side.Amp.autoName, Side.Amp);
        SideChooser.addOption(Side.Amp.autoName, Side.Amp);
        SideChooser.addOption(Side.Center.autoName, Side.Center);
        SideChooser.addOption(Side.Source.autoName, Side.Source);

        AutoModeChooser = new SendableChooser<>();
        AutoModeChooser.setDefaultOption(AutoMode.OneNote.autoName, AutoMode.OneNote);
        AutoModeChooser.addOption(AutoMode.OnlyShoot.autoName, AutoMode.OnlyShoot);
        AutoModeChooser.addOption(AutoMode.OneNote.autoName, AutoMode.OneNote);
        AutoModeChooser.addOption(AutoMode.TwoNote.autoName, AutoMode.TwoNote);
        AutoModeChooser.addOption(AutoMode.ThreeNote.autoName, AutoMode.ThreeNote);
        AutoModeChooser.addOption(AutoMode.FourNote.autoName, AutoMode.FourNote);
        AutoModeChooser.addOption(AutoMode.SubwooferDisrupt.autoName, AutoMode.SubwooferDisrupt);
        AutoModeChooser.addOption(AutoMode.SourceDisrupt.autoName, AutoMode.SourceDisrupt);

		AutoDirectionChooser = new SendableChooser<>();
		AutoDirectionChooser.setDefaultOption(AutoDirection.AmpDirection.autoName, AutoDirection.AmpDirection);
        AutoDirectionChooser.addOption(AutoDirection.AmpDirection.autoName, AutoDirection.AmpDirection);
        AutoDirectionChooser.addOption(AutoDirection.SourceDirection.autoName, AutoDirection.SourceDirection);

        RobotContainer.getShuffleboardTab().add(SideChooser);
        RobotContainer.getShuffleboardTab().add(AutoModeChooser);
		RobotContainer.getShuffleboardTab().add(AutoDirectionChooser);
    }

    @Override
    public Optional<Command> buildAutoCommand() {
        final Side side = SideChooser.getSelected();
        final AutoMode autoMode = AutoModeChooser.getSelected();
		final AutoDirection autoDirection = AutoDirectionChooser.getSelected();

        String autoName = autoMode.autoName + " " + side.autoName;

		if (side.autoName.equals("Center"))
		{
			autoName += " " + autoDirection.autoName;
		}

        if (autoMode.autoName.equals("Subwoofer Disrupt") || autoMode.autoName.equals("Source Disrupt"))
        {
            autoName = autoMode.autoName;
        }

        return Optional.of(getPathPlannerAuto(autoName));
    }

}
