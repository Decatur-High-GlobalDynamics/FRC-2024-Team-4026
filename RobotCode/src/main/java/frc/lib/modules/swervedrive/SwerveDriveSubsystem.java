package frc.lib.modules.swervedrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase
{
	private SwerveDriveOdometry swerveOdometry;
	private SwerveModule[] swerveMods;
	private Pigeon2 gyro;
	private final double MAX_MODULE_SPEED = SwerveConstants.MAX_SPEED;

	private double gyroOffset = 0;

	public SwerveDriveSubsystem()
	{
		gyro = new Pigeon2(SwervePorts.GYRO);

		gyro.configFactoryDefault();
		zeroGyro();

		setAngleOffsets(false);

		// swerve module initialization
		swerveMods = new SwerveModule[]
		{
				new SwerveModule(SwerveConstants.FRONT_LEFT, SwerveConstants.ModFL.Constants),
				new SwerveModule(SwerveConstants.FRONT_RIGHT, SwerveConstants.ModFR.constants),
				new SwerveModule(SwerveConstants.BACK_LEFT, SwerveConstants.ModBL.Constants),
				new SwerveModule(SwerveConstants.BACK_RIGHT, SwerveConstants.ModBR.Constants)
		};

		/*
		 * By pausing init for a second before setting module offsets, we avoid a bug with inverting
		 * motors. See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
		 */
		Timer.delay(1.0);
		resetModulesToAbsolute();

		// construct odometry (full robot position/incorporated module states)
		swerveOdometry = new SwerveDriveOdometry(SwerveConstants.SwerveKinematics, getYaw(),
				getModulePositions());

		configureAutoBuilder();
	}

	private void configureAutoBuilder()
	{
		HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
				new PIDConstants(SwerveConstants.DRIVE_KP, SwerveConstants.DRIVE_KI,
						SwerveConstants.DRIVE_KD),
				new PIDConstants(SwerveConstants.ANGLE_KP, SwerveConstants.ANGLE_KI,
						SwerveConstants.ANGLE_KD),
				SwerveConstants.MAX_SPEED, SwerveConstants.DRIVE_BASE_RADIUS_METERS,
				new ReplanningConfig());

		BooleanSupplier isRedAlliance = () ->
		{
			Optional<Alliance> alliance = DriverStation.getAlliance();
			return alliance.isPresent() && alliance.get() == Alliance.Red;
		};

		// Type::method gets a reference to the method. Type.method only allows us to run the method
		AutoBuilder.configureHolonomic(this::getPose, this::resetPose, this::getCurrentSpeeds,
				this::drive, pathFollowerConfig, isRedAlliance, this);
	}

	public void setAngleOffsets(boolean invert)
	{
		System.out.println("Setting angle offsets...");

		double[] offsets = SwerveConstants.ANGLE_OFFSETS;

		SwerveConstants.ModFL.angleOffset = Rotation2d.fromDegrees(offsets[0] - (invert ? 180 : 0));
		SwerveConstants.ModFR.angleOffset = Rotation2d.fromDegrees(offsets[1] - (invert ? 180 : 0));
		SwerveConstants.ModBL.angleOffset = Rotation2d.fromDegrees(offsets[2] - (invert ? 180 : 0));
		SwerveConstants.ModBR.angleOffset = Rotation2d.fromDegrees(offsets[3] - (invert ? 180 : 0));
	}

	// main driving method. translation is change in every direction
	public void drive(Translation2d translation, double rotation, boolean fieldRelative,
			boolean isOpenLoop)
	{
		drive(fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(),
						rotation, getYaw())
				: new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

	}

	private void drive(ChassisSpeeds speeds, boolean isOpenLoop)
	{
		SwerveModuleState[] swerveModuleStates = SwerveConstants.SwerveKinematics
				.toSwerveModuleStates(speeds);

		// Log module state
		for (int i = 0; i < swerveModuleStates.length; i++)
		{
			SwerveModuleState mod = swerveModuleStates[i];
			SmartDashboard.putNumber("Mod " + i + " Target Angle", mod.angle.getDegrees());
			SmartDashboard.putNumber("Mod " + i + " Target - CANCoder",
					mod.angle.getDegrees() - swerveMods[i].getCanCoder().getDegrees());
			SmartDashboard.putNumber("Mod " + i + " Target Speed", mod.speedMetersPerSecond);
		}

		// lowers module speeds to max attainable speed (avoids going above topspeed)
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_MODULE_SPEED);

		// sets modules to desired state (angle, speed)
		for (SwerveModule mod : swerveMods)
		{
			mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
		}
	}

	private void drive(ChassisSpeeds speeds)
	{
		drive(speeds, true);
	}

	/* Used by SwerveControllerCommand in Auto */
	public void setModuleStates(SwerveModuleState[] desiredStates)
	{
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_MODULE_SPEED);

		for (SwerveModule mod : swerveMods)
		{
			mod.setDesiredState(desiredStates[mod.moduleNumber], false);
		}
	}

	// gets position of robot on the field (odometry)
	public Pose2d getPose()
	{
		return swerveOdometry.getPoseMeters();
	}

	// resets odometry (position on field)
	public void resetPose(Pose2d pose)
	{
		swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
	}

	// returns array of a modules' states (angle, speed) for each one
	public SwerveModuleState[] getModuleStates()
	{
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (SwerveModule mod : swerveMods)
		{
			states[mod.moduleNumber] = mod.getState();
		}
		return states;
	}

	// returns module positions(for each individual module)
	public SwerveModulePosition[] getModulePositions()
	{
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (SwerveModule mod : swerveMods)
		{
			positions[mod.moduleNumber] = mod.getPosition();
		}
		return positions;
	}

	public SwerveModule[] getSwerveMods()
	{
		return swerveMods;
	}

	public void setGyro(double degrees)
	{
		System.out.println("Setting gyro to " + degrees + "...");
		gyroOffset = degrees - gyro.getYaw();
		zeroGyro();
	}

	public void zeroGyro()
	{
		System.out.println("Zeroing gyro");

		gyro.setYaw(0); // Used to setYaw(0);
	}

	public void setGyroOffset(double offset)
	{
		gyroOffset = offset;
	}

	/** Returns angle around vertical axis */
	public Rotation2d getYaw()
	{
		return (SwerveConstants.INVERT_GYRO)
				? Rotation2d.fromDegrees(360 - gyro.getYaw() + gyroOffset)
				: Rotation2d.fromDegrees(gyro.getYaw() + gyroOffset);
	}

	public void resetModulesToAbsolute()
	{
		for (SwerveModule mod : swerveMods)
		{
			mod.resetToAbsolute();
		}
	}

	@Override
	public void periodic()
	{
		swerveOdometry.update(getYaw(), getModulePositions());

		// smartdashboard logging per module
		for (SwerveModule mod : swerveMods)
		{
			SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder",
					mod.getCanCoder().getDegrees());
			SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated",
					mod.getPosition().angle.getDegrees());
			SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity",
					mod.getState().speedMetersPerSecond);
		}

		SmartDashboard.putNumber("Gyro Angle", getYaw().getDegrees());
	}

	public void resetEncoders()
	{
		for (SwerveModule mod : swerveMods)
		{
			mod.resetToAbsolute();
		}
	}

	public ChassisSpeeds getCurrentSpeeds()
	{
		return SwerveConstants.SwerveKinematics.toChassisSpeeds(getModuleStates());
	}
}