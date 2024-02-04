// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.core.ModeBasedSubsystem;
import frc.lib.core.util.CTREConfigs;

public class Robot extends TimedRobot
{

	private static Robot instance;

	private Optional<Command> autonomousCommand;
	private RobotContainer robotContainer;
	private CTREConfigs ctreConfigs;

	private ArrayList<ModeBasedSubsystem> subsystems = new ArrayList<>();

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit()
	{
		if (instance != null)
			System.err.println("WARNING: Robot instance already exists!");
		instance = this;

		ctreConfigs = new CTREConfigs();

		// Instantiate our RobotContainer. This will perform all our button bindings,
		// and put our
		// autonomous chooser on the dashboard.
		robotContainer = new RobotContainer();
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
	 * that you want ran during disabled, autonomous, teleoperated and test.
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic()
	{
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit()
	{
		for (ModeBasedSubsystem subsystem : subsystems)
		{
			subsystem.disabledInit();
		}
	}

	@Override
	public void disabledPeriodic()
	{}

	/**
	 * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit()
	{
		for (ModeBasedSubsystem subsystem : subsystems)
		{
			subsystem.autonomousInit();
		}

		autonomousCommand = Autonomous.getAutoCommand();
		Autonomous.close();
		// schedule the autonomous command (example)
		if (autonomousCommand.isPresent())
		{
			autonomousCommand.get().schedule();
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic()
	{}

	@Override
	public void teleopInit()
	{
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand.isPresent())
		{
			autonomousCommand.get().cancel();
		}

		for (ModeBasedSubsystem subsystem : subsystems)
		{
			subsystem.teleopInit();
		}
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic()
	{}

	@Override
	public void testInit()
	{
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();

		for (ModeBasedSubsystem subsystem : subsystems)
		{
			subsystem.testInit();
		}
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic()
	{}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit()
	{}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic()
	{}

	public static void addSubsystem(ModeBasedSubsystem subsystem)
	{
		instance.subsystems.add(subsystem);
	}

	public static CTREConfigs getCtreConfigs()
	{
		return instance.ctreConfigs;
	}
}
