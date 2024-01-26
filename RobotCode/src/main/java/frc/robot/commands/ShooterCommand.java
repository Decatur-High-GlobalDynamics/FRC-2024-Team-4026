package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command
{
	
	// Initializes the subsystem object
	private final ShooterSubsystem Shooter;

	public ShooterCommand(ShooterSubsystem shoot)
	{

		Shooter = shoot;
		addRequirements(Shooter);
	}

	public void execute()
	{

		// Spins up the motor
		Shooter.setShooterMotorPower(1.0, "joystick said to shoot");
		// If-statement to see if motor is spun up
		if (Shooter.getShooterMotorPower() >= 0.95)
		{
			Shooter.setFeedMotorPower(1.0, "motor is spun");
		}
		else
		{
			Shooter.setFeedMotorPower(0, "motor is not spun");
		}
	}

	public void end(boolean interrupted)
	{
		Shooter.setShooterMotorPower(0.25, "command is over");
		Shooter.setFeedMotorPower(0, "command is over");
	}
}
