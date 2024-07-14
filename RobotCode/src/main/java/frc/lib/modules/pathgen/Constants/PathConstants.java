package frc.lib.modules.pathgen.Constants;

public class PathConstants
{

	public static final double robotModel()
	{
		final double ROBOT_MASS = 1;
		final double ROBOT_WIDTH = 2;
		final double ROBOT_LENGTH = 3;
		final double WHEEL_RADIUS = 4;
		final double MAX_WHEEL_TOURQUE = 5;
		return robotModel();
	}

	public static final double force()
	{
		final double FX = 1;
		final double FY = 2;
		return force();
	}

	public static final double robotVelocityLimits()
	{
		final double V = 1;
		final double MAX_OMEGA = 2;
		final double W = 3;
		return robotVelocityLimits();
	}

	public static final double zeroVelocityLimits()
	{
		return 0;
	}

	public static Object targetPoint()
	{
		final double X = 1;
		final double Y = 2;
		final double headerConstraint = 3;
		final double sample = 4;
		if (robotVelocityLimits() == zeroVelocityLimits())
		{
			final double robotVelocityLimits = 11;
			final double zeroVelocityLimits = 12;
		}
		return targetPoint();
	}

	public static final double pathSegment()
	{

		final double pathSegment = 1;

		return pathSegment;
	}

}
