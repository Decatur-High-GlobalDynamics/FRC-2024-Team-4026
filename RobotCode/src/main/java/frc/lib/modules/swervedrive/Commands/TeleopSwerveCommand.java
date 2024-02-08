package frc.lib.modules.swervedrive.Commands;

import frc.lib.modules.swervedrive.SwerveConstants;
import frc.lib.modules.swervedrive.SwerveDriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerveCommand extends Command
{
	private SwerveDriveSubsystem s_Swerve;
	private DoubleSupplier translationSup;
	private DoubleSupplier strafeSup;
	private DoubleSupplier rotationSup;
	private BooleanSupplier slowSpeedSupplier, fastSpeedSupplier;

	public TeleopSwerveCommand(SwerveDriveSubsystem s_Swerve, DoubleSupplier translationSup,
			DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier slowSpeedSupplier,
			BooleanSupplier fastSpeedSupplier)
	{
		this.s_Swerve = s_Swerve;
		addRequirements(s_Swerve);

		this.translationSup = translationSup;
		this.strafeSup = strafeSup;
		this.rotationSup = rotationSup;

		this.slowSpeedSupplier = slowSpeedSupplier;
		this.fastSpeedSupplier = fastSpeedSupplier;
	}

	private double getSpeed()
	{
		if (slowSpeedSupplier.getAsBoolean())
			return SwerveConstants.SLOW_SPEED;
		if (fastSpeedSupplier.getAsBoolean())
			return SwerveConstants.FAST_SPEED;
		return SwerveConstants.NORMAL_SPEED;
	}

	@Override
	public void execute()
	{
		/* Get Values, Deadband */
		double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(),
				SwerveConstants.JOYSTICK_DEADBAND);
		double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(),
				SwerveConstants.JOYSTICK_DEADBAND);
		double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(),
				SwerveConstants.JOYSTICK_DEADBAND);

		double speed = getSpeed();
		SmartDashboard.putNumber("Swerve Speed", speed);

		/* Drive */
		s_Swerve.drive(
				new Translation2d(translationVal, strafeVal)
						.times(speed * SwerveConstants.BASE_DRIVE_SPEED),
				rotationVal * speed * SwerveConstants.BASE_TURN_SPEED, true, // field relative is
																				// always on
				true);
	}
}