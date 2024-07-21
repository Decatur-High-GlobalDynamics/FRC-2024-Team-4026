package frc.lib.modules.swervedrive.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.lib.modules.swervedrive.SwerveDriveSubsystem;

/**
 * Rotates the chassis towards the a specified rotation but allows the driver to control translation
 * and strafe
 */
public class TeleopAimSwerveToPositionCommand extends TeleopSwerveCommand
{

	private final SwerveDriveSubsystem Swerve;

	public TeleopAimSwerveToPositionCommand(SwerveDriveSubsystem swerve,
			DoubleSupplier translationSup, DoubleSupplier strafeSup,
			BooleanSupplier slowSpeedSupplier, double desiredRotation)
	{
		super(swerve, translationSup, strafeSup,
				() -> swerve.getRotationalVelocityToAngle(desiredRotation), slowSpeedSupplier);

		Swerve = swerve;
	}

	@Override
	public void execute()
	{
		super.execute();
	}

	@Override
	public void end(boolean interrupted)
	{
		Swerve.setRotationController(null);
	}

}
