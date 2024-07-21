package frc.lib.modules.swervedrive.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.modules.swervedrive.SwerveDriveSubsystem;

public class ZeroGyroCommand extends InstantCommand 
{
	private final SwerveDriveSubsystem Swerve;

	public ZeroGyroCommand(final SwerveDriveSubsystem Swerve)
	{
		this.Swerve = Swerve;
	}

	@Override
	public void initialize() 
	{
		Swerve.zeroGyro();
	}
}
