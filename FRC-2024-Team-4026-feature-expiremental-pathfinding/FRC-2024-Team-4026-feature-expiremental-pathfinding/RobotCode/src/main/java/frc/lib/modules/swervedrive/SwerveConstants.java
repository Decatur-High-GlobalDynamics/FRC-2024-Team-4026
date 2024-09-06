// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.modules.swervedrive;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class SwerveConstants
{

	public static final double JOYSTICK_DEADBAND = 0.1;

	public static final class CanSparkMaxPeriodicRates
	{
		// In ms
		public static final int LOW_INTERVAL = 30; // Originally 20
		public static final int MID_INTERVAL = 100; // Originally 50
		public static final int HIGH_INTERVAL = 1000; // Originally 500
	}

	public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

	/* Drivetrain Constants */
	public static final double TRACK_WIDTH = Units.inchesToMeters(19); // will test
																		// when
																		// testing PID
	public static final double WHEEL_BASE = Units.inchesToMeters(19); // will test when
																		// testing
																		// PID
	public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.75);
	public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

	// Find this for the robot!
	public static final double DRIVE_BASE_RADIUS_METERS = 0.33;

	/*
	 * Swerve Kinematics No need to ever change this unless you are not doing a traditional
	 * rectangular/square 4 module swerve
	 */
	public static final SwerveDriveKinematics SwerveKinematics = new SwerveDriveKinematics(
			new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
			new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
			new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
			new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

	public static final double VOLTAGE_COMP_TARGET = 12.0;

	/* Module Gear Ratios */
	public static final double DRIVE_GEAR_RATIO = 8.14;
	public static final double ANGLE_GEAR_RATIO = 150.0 / 7.0;

	/* Motor Inverts */
	public static final boolean ANGLE_MOTOR_INVERT = true; // check when testing
	public static final boolean DRIVE_MOTOR_INVERT = false;

	/* Angle Encoder Invert */
	public static final SensorDirectionValue CANCODER_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;

	/* Swerve Current Limiting */
	public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
	public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
	public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
	public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

	public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
	public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
	public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
	public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

	/*
	 * These values are used by the drive falcon to ramp in open loop and closed loop driving. We
	 * found a small open loop ramp (0.25) helps with tread wear, tipping, etc
	 */
	public static final double OPEN_LOOP_RAMP = 0.25;
	public static final double CLOSED_LOOP_RAMP = 0.25;

	/* Angle Motor PID Values */
	public static final double ANGLE_KP = 0.15; // must find/double check all of these
													// in sysid
	public static final double ANGLE_KI = 0.0;
	public static final double ANGLE_KD = 0.05;
	public static final double ANGLE_KF = 0.0;

	/* Drive Motor PID Values */
	public static final double DRIVE_KP = 0.03; // TODO: This must be tuned to specific
												// robot
	public static final double DRIVE_KI = 0.0;
	public static final double DRIVE_KD = 0.001;

	/*
	 * Drive Motor Characterization Values Divide SYSID values by 12 to convert from volts to
	 * percent output for CTRE
	 */
	public static final double DRIVE_KS = 0.045; // TODO: This must be tuned to
														// specific
														// robot
	public static final double DRIVE_KV = 0.01;
	public static final double DRIVE_KA = 0.0025;

	/* Angle Motor Conversion Factors */
	public static final double ANGLE_CONVERSION_FACTOR = 360.0 / ANGLE_GEAR_RATIO;

	/* Swerve Profiling Values */
	/* Meters per Second */
	public static final double SLOW_SPEED = 1.5;
	public static final double AUTO_SPEED = 3.7;
	public static final double NORMAL_SPEED = 3.7;
	public static final double FAST_SPEED = 3.7;
	public static final double MAX_SPEED = 3.7; //TODO: This must be tuned to
												// specific
												// robot
	/** Radians per Second */
	public static final double MAX_ANGULAR_VELOCITY = 9.42478; //TODO: This must be
															// tuned to
															// specific robot

	/** Radians per Second Squared */
	public static final double MAX_ANGULAR_ACCELERATION = 12.5664; //TODO: This must be
															   // tuned to
															   // specific robot

	/* Neutral Modes */
	public static final IdleMode ANGLE_NEUTRAL_MODE = IdleMode.kCoast;
	public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

	public static final boolean DEFAULT_ANGLE_INVERT = false;

	public static final double[] ANGLE_OFFSETS = new double[]
	{
			151.35, // FL
			80.68, // FR
			27.77, // BL
			119.44 // BR
	};

	public static final int FRONT_LEFT = 0;
	public static final int FRONT_RIGHT = 1;
	public static final int BACK_LEFT = 2;
	public static final int BACK_RIGHT = 3;

	// Velocity measurement constants
	/** TODO: Needs to checked */
	public static final int DRIVE_MOTOR_TICKS_PER_ROTATION = 2048;

	/* Module Specific Constants */
	/* Front Left Module - Module 0 */
	public static final class ModFL
	{ // TODO: This must be tuned to specific robot
		public static final int DRIVE_MOTOR_PORT = SwervePorts.MOD0_DRIVEMOTOR;
		public static final int ANGLE_MOTOR_PORT = SwervePorts.MOD0_ANGLEMOTOR;
		public static final int CANCODER_PORT = SwervePorts.MOD0_CANCODER;
		public static Rotation2d angleOffset = Rotation2d
				.fromDegrees(ANGLE_OFFSETS[0] - (DEFAULT_ANGLE_INVERT ? 180 : 0));;
		public static final SwerveModuleConstants Constants = new SwerveModuleConstants(
				DRIVE_MOTOR_PORT, ANGLE_MOTOR_PORT, CANCODER_PORT, angleOffset);
	}

	/* Front Right Module - Module 1 */
	public static final class ModFR
	{ // TODO: This must be tuned to specific robot
		public static final int DRIVE_MOTOR_PORT = SwervePorts.MOD1_DRIVEMOTOR;
		public static final int ANGLE_MOTOR_PORT = SwervePorts.MOD1_ANGLEMOTOR;
		public static final int CANCODER_PORT = SwervePorts.MOD1_CANCODER;
		public static Rotation2d angleOffset = Rotation2d
				.fromDegrees(ANGLE_OFFSETS[1] - (DEFAULT_ANGLE_INVERT ? 180 : 0));;
		public static final SwerveModuleConstants constants = new SwerveModuleConstants(
				DRIVE_MOTOR_PORT, ANGLE_MOTOR_PORT, CANCODER_PORT, angleOffset);
	}

	/* Back Left Module - Module 2 */
	public static final class ModBL
	{ // TODO: This must be tuned to specific robot
		public static final int DRIVE_MOTOR_PORT = SwervePorts.MOD2_DRIVEMOTOR;
		public static final int ANGLE_MOTOR_PORT = SwervePorts.MOD2_ANGLEMOTOR;
		public static final int CANCODER_PORT = SwervePorts.MOD2_CANCODER;
		public static Rotation2d angleOffset = Rotation2d
				.fromDegrees(ANGLE_OFFSETS[2] - (DEFAULT_ANGLE_INVERT ? 180 : 0));;
		public static final SwerveModuleConstants Constants = new SwerveModuleConstants(
				DRIVE_MOTOR_PORT, ANGLE_MOTOR_PORT, CANCODER_PORT, angleOffset);
	}

	/* Back Right Module - Module 3 */
	public static final class ModBR
	{ // TODO: This must be tuned to specific robot
		public static final int DRIVE_MOTOR_PORT = SwervePorts.MOD3_DRIVEMOTOR;
		public static final int ANGLE_MOTOR_PORT = SwervePorts.MOD3_ANGLEMOTOR;
		public static final int CANCODER_PORT = SwervePorts.MOD3_CANCODER;
		public static Rotation2d angleOffset = Rotation2d
				.fromDegrees(ANGLE_OFFSETS[3] - (DEFAULT_ANGLE_INVERT ? 180 : 0));;
		public static final SwerveModuleConstants Constants = new SwerveModuleConstants(
				DRIVE_MOTOR_PORT, ANGLE_MOTOR_PORT, CANCODER_PORT, angleOffset);
	}

	public static final class AutoConstants
	{
		// TODO: The below constants are used in the example auto, and must be
		// tuned to specific robot
		public static final double MAX_SPEED = 3.7; // In meters per second
		public static final double MAX_ACCELERATION = 3; // In meters per second squared
		public static final double MAX_ANGULAR_SPEED = 1.5*Math.PI; // In radians per
																// second
		public static final double MAX_ANGULAR_ACCELERATION = 2*Math.PI; // In radians
																		// per
																		// second
																		// squared
		public static final double PX_CONTROLLER = 1;
		public static final double PY_CONTROLLER = 1;
		public static final double PTHETA_CONTROLLER = 1;

		/* Constraint for the motion profilied robot angle controller */
		public static final TrapezoidProfile.Constraints ThetaControllerConstraints = new TrapezoidProfile.Constraints(
				MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION);

		public static final PathConstraints PathConstraints = new PathConstraints(MAX_SPEED,
				MAX_ACCELERATION, MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION);
	}

	// Constraints for robot angle aiming PID controller
	public static final double ANGULAR_AIMING_KP = 0.3;
	public static final double ANGULAR_AIMING_KI = 0;
	public static final double ANGULAR_AIMING_KD = 0.02;

	public static final double ANGULER_AIMING_DEADBAND = 0.05;
	
	/**  */

	/* Constraint for the motion profilied robot angle controller */
	public static final TrapezoidProfile.Constraints ANGULAR_VELOCITY_CONSTRAINTS = new TrapezoidProfile.Constraints(
			3.7, 7.4);

	/** Rotation to face blue amp when on the blue alliance in radians */
	public static final double AMP_ROTATION = -Math.PI;

	

}
