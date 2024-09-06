package frc.lib.modules.swervedrive.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.modules.swervedrive.SwerveConstants;
import frc.lib.modules.swervedrive.SwerveDriveSubsystem;

/**
 * Includes both movement and rotation. Rotation is in degrees for this command.
 */
public class DriveDistanceAuto extends Command
{

	private final SwerveDriveSubsystem Swerve;

	private double speed, distanceX, distanceY, targetPositionX, targetPositionY, rotSpeed,
			targetRot;

	private final double ROT_DEADBAND = 1;

	/**
	 * Includes both movement and rotation.
	 * 
	 * @param distanceX in meters
	 * @param distanceY in meters
	 * @param targetRot in degrees
	 */
	public DriveDistanceAuto(double distanceX, double distanceY, double speed, double rotSpeed,
			double targetRot, SwerveDriveSubsystem swerve)
	{
		this.distanceX = distanceX;
		this.distanceY = distanceY;

		this.rotSpeed = rotSpeed;
		this.targetRot = targetRot;

		this.speed = speed;
		this.Swerve = swerve;

		addRequirements(swerve);
	}

	/**
	 * Includes both movement in both the x and y axes.
	 * 
	 * @param distanceX in meters
	 * @param distanceY in meters
	 */
	public DriveDistanceAuto(double distanceX, double distanceY, double speed,
			SwerveDriveSubsystem swerve)
	{
		this(distanceX, distanceY, speed, 0, 0, swerve);
	}

	/**
	 * Includes movement in the x axis.
	 * 
	 * @param distanceX in meters
	 */
	public DriveDistanceAuto(double distanceX, double speed, SwerveDriveSubsystem swerve)
	{
		this(distanceX, 0, speed, 0, 0, swerve);
	}

	@Override
	public void initialize()
	{
		targetPositionX = Swerve.getPose().getX() + distanceX;
		targetPositionY = Swerve.getPose().getY() + distanceY;
	}

	@Override
	public void execute()
	{

		// Update distanceX & distanceY to reflect the remaining distance
		distanceX = targetPositionX - Swerve.getPose().getX();
		distanceY = targetPositionY - Swerve.getPose().getY();

		// Update SmartDashboard
		SmartDashboard.putString("Auto Drive Target",
				"(" + targetPositionX + ", " + targetPositionY + ")");
		SmartDashboard.putString("Auto Drive Distance Remaining",
				"(" + distanceX + ", " + distanceY + ")");
		SmartDashboard.putNumber("Auto Drive Rot Target", targetRot);
		SmartDashboard.putString("Swerve Pose",
				"(" + Swerve.getPose().getX() + ", " + Swerve.getPose().getY() + ")");

		// Calculate speed
		Translation2d speedVector = new Translation2d(
				distanceX > 0 ? Math.min(speed * SwerveConstants.AUTO_SPEED, distanceX)
						: Math.max(-speed * SwerveConstants.AUTO_SPEED, distanceX),
				distanceY > 0 ? Math.min(speed * SwerveConstants.AUTO_SPEED, distanceY)
						: Math.max(-speed * SwerveConstants.AUTO_SPEED, distanceY));

		// Calculate rotation
		double currentRotation = Swerve.getYaw().getDegrees(),
				rotDiff = targetRot - currentRotation;

		// Make sure the rotation is between -180 and 180
		if (rotDiff > 180)
			rotDiff -= 360;
		else if (rotDiff < -180)
			rotDiff += 360;

		// Calculate rotation speed
		double rotSpeed = rotDiff > 0 ? Math.min(this.rotSpeed, rotDiff)
				: Math.max(-this.rotSpeed, rotDiff);

		Swerve.drive(speedVector, rotSpeed, true, true);
	}

	@Override
	public boolean isFinished()
	{
		// Check if both x and y are finished

		boolean xFinished = false;
		if (distanceX > 0)
			xFinished = Swerve.getPose().getX() > targetPositionX;
		else
			xFinished = Swerve.getPose().getX() < targetPositionX;

		boolean yFinished = false;
		if (distanceY > 0)
			yFinished = Swerve.getPose().getY() > targetPositionY;
		else
			yFinished = Swerve.getPose().getY() < targetPositionY;

		boolean rotFinished = Math.abs(targetRot - Swerve.getYaw().getDegrees()) < ROT_DEADBAND;

		return xFinished && yFinished && rotFinished;
	}
}
