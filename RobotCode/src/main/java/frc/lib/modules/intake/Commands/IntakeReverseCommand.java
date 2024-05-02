package frc.lib.modules.intake.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IndexerConstants;
import frc.lib.modules.intake.IntakeConstants;
import frc.lib.modules.shooter.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.lib.modules.intake.IntakeSubsystem;
import frc.lib.modules.shooter.ShooterSubsystem;

public class IntakeReverseCommand extends Command {
    
    private IntakeSubsystem intake;
    private IndexerSubsystem indexer;
    private ShooterSubsystem shooter;

    public IntakeReverseCommand(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter)
    {
        this.intake = intake;
        this.indexer = indexer;
        this.shooter = shooter;
        addRequirements(intake, indexer, shooter);
    }

    @Override
    public void initialize()
    {
        intake.setDesiredVelocity(IntakeConstants.INTAKE_REVERSE_VELOCITY);
		// intake.setDesiredRotation(IntakeConstants.INTAKE_DEPLOYED_ROTATION);
        indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_REVERSE_VELOCITY);
        shooter.setShooterMotorVelocity(ShooterConstants.SHOOTER_REVERSE_VELOCITY);
    }

    @Override
    public void end(boolean isFinished)
    {
        intake.setDesiredVelocity(IntakeConstants.INTAKE_REST_VELOCITY);
		intake.setDesiredRotation(IntakeConstants.INTAKE_RETRACTED_ROTATION);
        indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_REST_VELOCITY);
        shooter.setShooterMotorVelocity(ShooterConstants.SHOOTER_REST_VELOCITY);
    }

}
