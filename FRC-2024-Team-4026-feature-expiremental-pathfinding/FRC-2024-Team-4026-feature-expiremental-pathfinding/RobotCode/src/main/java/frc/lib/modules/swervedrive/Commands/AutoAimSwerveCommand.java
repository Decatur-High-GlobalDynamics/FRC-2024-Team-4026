package frc.lib.modules.swervedrive.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.modules.swervedrive.SwerveDriveSubsystem;
import frc.lib.modules.shooter.ShooterConstants;
import frc.lib.core.ILogSource;

/**
 * Rotates the chassis towards the speaker. Intended to work with PathPlanner paths. Will end once
 * the note is fired. See {@link ShooterConstants#SHOOT_TIME} for the time to wait after shooting.
 */
public class AutoAimSwerveCommand extends Command implements ILogSource
{

	private final SwerveDriveSubsystem Swerve;
	private double angle;

	public AutoAimSwerveCommand(SwerveDriveSubsystem swerve, double angle)
	{
		Swerve = swerve;
		this.angle = angle;
	}

	@Override
	public void initialize()
	{
		logInfo("Starting AutoAimSwerveCommand");
	}

	@Override
	public void execute()
	{
		System.out.println(angle);
		Swerve.drive(new Translation2d(), Swerve.getRotationalVelocityToAngle(angle) * 3.7 * 3.7, true, false);
	}

	@Override
	public void end(boolean interrupted)
	{
		logInfo("Ending AutoAimSwerveCommand");
		Swerve.drive(new Translation2d(), 0, true, false);
	}

	@Override
	public boolean isFinished()
	{
		return Swerve.atRotation(angle);
	}
}