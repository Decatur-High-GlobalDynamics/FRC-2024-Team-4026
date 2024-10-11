package frc.lib.modules.shooter.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.modules.leds.TeamColor;
import frc.lib.modules.leds.LedSubsystem;
import frc.lib.modules.shooter.ShooterSubsystem;
import frc.lib.modules.shootermount.ShooterMountConstants;
import frc.lib.modules.shootermount.ShooterMountSubsystem;
import frc.lib.modules.shooter.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.RobotContainer;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.SwerveConstants;


public class ShootFromDistance extends Command {
	// Initializes the subsystem objects
	private ShooterSubsystem shooter;
	private IndexerSubsystem indexer;
	private ShooterMountSubsystem shooterMount;
	private LedSubsystem leds;

	private double targetShooterMountPosition;

	public ShootFromDistance(ShooterSubsystem shooter, IndexerSubsystem indexer, ShooterMountSubsystem shooterMount,
			LedSubsystem leds) {
		this.shooter = shooter;
		this.indexer = indexer;
		this.shooterMount = shooterMount;
		this.leds = leds;

		addRequirements(shooter, indexer, shooterMount, leds);
	}

	@Override
	public void execute() {
		targetShooterMountPosition = shooterMount.getShooterMountAngleTreeMap().get(RobotContainer.getDistanceToSpeaker());

		// Spins up the motor
		shooter.setShooterMotorVelocity(ShooterConstants.SHOOTER_SPEAKER_VELOCITY);
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
}
