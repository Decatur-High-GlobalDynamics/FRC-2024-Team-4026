package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase
{
	// makes the subsystem object
	private ShooterSubsystem shooter;

	public ShooterCommand(ShooterSubsystem shoot)
	{
		// constructor i guess
		shooter = shoot;
		addRequirements(shooter);
	}
	// execute command
	public void execute()
	{
		
		// spins up the motor
		shooter.setShooterMotorPower(1.0, "joystick said to shoot");
		// if statement  to see if motor is spun up
		if (shooter.getShooterMotorPower() >= 0.95)
		{
			shooter.setFeedMotorPower(1.0, "motor is spun");
		}
		else
		{
			shooter.setFeedMotorPower(0, "motor is not spun");
		}
	}
	// the end command
	public void end(boolean interrupted)
	{
		shooter.setShooterMotorPower(0.25, "command is over");
		shooter.setFeedMotorPower(0, "command is over");
	}
}
