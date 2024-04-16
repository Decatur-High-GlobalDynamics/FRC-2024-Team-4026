package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.core.util.TeamCountdown;
import frc.lib.modules.leds.Color;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterMountSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class IntakeCommand extends Command {
	private IntakeSubsystem intake;
	private IndexerSubsystem indexer;
	private ShooterMountSubsystem shooterMount;
	private ShooterSubsystem shooter;
	private LedSubsystem leds;
	private State state;

	private TeamCountdown countdown;

	public IntakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer,
			ShooterMountSubsystem shooterMount, ShooterSubsystem shooter, LedSubsystem leds) {
		this.intake = intake;
		this.indexer = indexer;
		this.shooterMount = shooterMount;
		this.shooter = shooter;
		this.leds = leds;

		state = State.FORWARD;
		addRequirements(intake, indexer, shooterMount, shooter);
	}

	enum State {
		FORWARD, REVERSE, DONE
	}

	@Override
	public void initialize() {
		state = State.FORWARD;

		intake.setDesiredRotation(IntakeConstants.INTAKE_DEPLOYED_ROTATION);
		intake.setDesiredVelocity(IntakeConstants.INTAKE_DEPLOYED_VELOCITY);
		indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_INTAKE_VELOCITY);
		shooterMount.setTargetRotation(0);
		shooter.setShooterMotorVelocity(ShooterConstants.SHOOTER_REST_VELOCITY);
	}

	@Override
	public void execute() {
		if (indexer.hasNote() && state == State.FORWARD) {
			intake.setDesiredRotation(IntakeConstants.INTAKE_RETRACTED_ROTATION);
			intake.setDesiredVelocity(IntakeConstants.INTAKE_REST_VELOCITY);
			indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_REVERSE_VELOCITY);
			if (leds != null)
				leds.flashAllPixels(Color.Yellow);

			state = State.REVERSE;

			countdown = new TeamCountdown(200);
		}
		if (countdown != null && countdown.isDone()) {
			state = State.DONE;
			countdown = null;
		}
	}

	@Override
	public void end(boolean stop) {
		intake.setDesiredRotation(IntakeConstants.INTAKE_RETRACTED_ROTATION);
		intake.setDesiredVelocity(IntakeConstants.INTAKE_REST_VELOCITY);
		indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_REST_VELOCITY);
	}

	@Override
	public boolean isFinished() {
		return state == State.DONE;
	}
}
