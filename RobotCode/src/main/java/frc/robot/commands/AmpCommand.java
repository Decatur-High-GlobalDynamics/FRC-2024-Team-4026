package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.core.util.TeamCountdown;
import frc.lib.modules.leds.TeamColor;
import frc.lib.modules.leds.LedSubsystem;
import frc.robot.constants.IndexerConstants;
import frc.lib.modules.shooter.ShooterConstants;
import frc.lib.modules.shootermount.ShooterMountConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterMountSubsystem;
import frc.lib.modules.shooter.ShooterSubsystem;

public class AmpCommand extends Command
{

	private ShooterSubsystem shooter;
	private IndexerSubsystem indexer;
	private ShooterMountSubsystem shooterMount;
	private LedSubsystem leds;

	private TeamCountdown startCountdown, finishCountdown;

	public AmpCommand(ShooterMountSubsystem shooterMount, ShooterSubsystem shooter,
			IndexerSubsystem indexer, LedSubsystem leds)
	{
		this.shooterMount = shooterMount;
		this.shooter = shooter;
		this.indexer = indexer;
		this.leds = leds;

		addRequirements(shooterMount, shooter, indexer, leds);
	}

	@Override
	public void initialize()
	{
		shooterMount
				.setTargetRotation(ShooterMountConstants.SHOOTER_MOUNT_INITIAL_AMP_ANGLE_OFFSET);
		shooter.setShooterMotorVelocity(ShooterConstants.SHOOTER_AMP_VELOCITY);

		startCountdown = new TeamCountdown(500);
		finishCountdown = null;
	}

	@Override
	public void execute()
	{
		if (startCountdown != null && startCountdown.isDone())
		{
			indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_AMP_VELOCITY);
			startCountdown = null;
			finishCountdown = new TeamCountdown(100);
		}

		if (finishCountdown != null && finishCountdown.isDone())
		{
			shooterMount
					.setTargetRotation(ShooterMountConstants.SHOOTER_MOUNT_ENDING_AMP_ANGLE_OFFSET);
			finishCountdown = null;
		}
	}

	@Override
	public void end(boolean isFinished)
	{
		shooterMount.setTargetRotation(0);
		shooter.setShooterMotorVelocity(ShooterConstants.SHOOTER_REST_VELOCITY);
		indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_REST_VELOCITY);
		leds.flashAllPixels(TeamColor.Blue, 5);
	}

}
