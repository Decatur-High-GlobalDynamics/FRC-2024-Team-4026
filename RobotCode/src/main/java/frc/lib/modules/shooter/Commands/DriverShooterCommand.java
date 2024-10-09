package frc.lib.modules.shooter.Commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.lib.modules.leds.TeamColor;
import frc.lib.modules.leds.LedSubsystem;
import frc.lib.modules.shooter.ShooterSubsystem;
import frc.lib.modules.shootermount.ShooterMountConstants;
import frc.lib.modules.shootermount.ShooterMountSubsystem;
import frc.lib.modules.shooter.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.lib.modules.indexer.IndexerConstants;

public class DriverShooterCommand extends Command {
	// Initializes the subsystem objects
	private ShooterSubsystem shooter;
	private IndexerSubsystem indexer;
	private ShooterMountSubsystem shooterMount;
	private LedSubsystem leds;

	private double targetShooterMountPosition;

	private double desiredShooterVelocity;

	private boolean endAutomatically;

	public DriverShooterCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, ShooterMountSubsystem shooterMount,
			LedSubsystem leds, double desiredShooterVelocity, double targetShooterMountPosition, boolean endAutomatically) {
		this.desiredShooterVelocity = desiredShooterVelocity;
		this.endAutomatically = endAutomatically;

		this.targetShooterMountPosition = targetShooterMountPosition;

		this.shooter = shooter;
		this.indexer = indexer;
		this.shooterMount = shooterMount;
		this.leds = leds;
		addRequirements(shooter, indexer, shooterMount, leds);
	}

	@Override
	public void execute() {
		// Spins up the motor
		shooter.setShooterMotorVelocity(desiredShooterVelocity);
		shooterMount.setTargetRotation(targetShooterMountPosition);

		// If-statement to see if motor is spun up
		if (shooter.isUpToSpeed() && shooterMount.isAtTargetRotation()) {
			indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_SHOOT_VELOCITY);
		}

		if (!indexer.hasNote() && leds != null) {
			leds.flashAllPixels(TeamColor.Blue, 5);
		}
	}

	@Override
	public void end(boolean interrupted) {
		shooter.setShooterMotorVelocity(ShooterConstants.SHOOTER_REST_VELOCITY);
		indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_REST_VELOCITY);
		shooterMount.setTargetRotation(0);
	}

	@Override
	public boolean isFinished() {
		return endAutomatically && !indexer.hasNote();
	}
}
