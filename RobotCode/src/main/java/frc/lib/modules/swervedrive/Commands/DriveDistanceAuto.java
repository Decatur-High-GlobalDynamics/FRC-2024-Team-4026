package frc.lib.modules.swervedrive.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.modules.swervedrive.SwerveConstants;
import frc.lib.modules.swervedrive.SwerveDriveSubsystem;

/**
 * @deprecated Use PathPlanner instead
 */
@Deprecated
public class DriveDistanceAuto extends Command
{
	DriveDistanceAuto driveDistanceAuto;
	private double distance;

	SwerveDriveSubsystem s_Swerve;

	private double speed;

	private double targetPosition;

	public DriveDistanceAuto(double distance, double speed, SwerveDriveSubsystem s_Swerve)
	{
		this.distance = distance;
		this.speed = speed;
		this.s_Swerve = s_Swerve;

		addRequirements(s_Swerve);
	}

	@Override
	public void initialize()
	{
		targetPosition = s_Swerve.getPose().getX() + distance;
	}

	@Override
	public void execute()
	{
		SmartDashboard.putNumber("Auto Drive Target X", targetPosition);
		SmartDashboard.putNumber("Swerve Pose X", s_Swerve.getPose().getX());

		s_Swerve.drive(new Translation2d(-speed, 0).times(SwerveConstants.AUTO_SPEED), 0,
				/*
				 * !robotCentricSup . getAsBoolean (),
				 */ true, // field
							// relative
							// is
							// always
							// on
				true);
	}

	@Override
	public boolean isFinished()
	{
		if (distance > 0)
			return (s_Swerve.getPose().getX() > targetPosition);
		else
			return (s_Swerve.getPose().getX() < targetPosition);
	}
}
