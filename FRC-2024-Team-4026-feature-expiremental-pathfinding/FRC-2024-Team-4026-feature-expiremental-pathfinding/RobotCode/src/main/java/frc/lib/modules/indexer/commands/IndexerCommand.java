package frc.lib.modules.indexer.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.modules.indexer.IndexerConstants;
import frc.lib.modules.indexer.IndexerSubsystem;;

public class IndexerCommand extends Command
{
	private IndexerSubsystem indexer;

	public IndexerCommand(IndexerSubsystem indexer)
	{
		this.indexer = indexer;

		addRequirements(indexer);
	}

	@Override
	public void initialize()
	{
		indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_SHOOT_VELOCITY);
	}

	@Override
	public void end(boolean stop)
	{
		indexer.setIndexerMotorVelocity(IndexerConstants.INDEXER_REST_VELOCITY);
	}
}
