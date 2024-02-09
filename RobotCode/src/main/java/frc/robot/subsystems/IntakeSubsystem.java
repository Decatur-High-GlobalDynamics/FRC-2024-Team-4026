package frc.robot.subsystems;

import frc.robot.constants.Ports;

// import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.core.motors.TeamSparkMAX;

public class IntakeSubsystem extends SubsystemBase
{
    // The intake raiser motors raise and lower the intake body.
    // Intake toggle motor turns on or off the motors that actually take in notes (game pieces).
    // Internally, it is known as INTAKE_ON_MOTOR, but I renamed it for the sake of clarity.
    // The right motor is the main motor, and the left motor is the follow motor.
    private final TeamSparkMAX INTAKE_RAISER_MOTOR_LEFT, INTAKE_RAISER_MOTOR_RIGHT,
            INTAKE_TOGGLE_MOTOR;
    public boolean isLowered = false;

    /*
     * Put as a constant, I don't think it is a value that has to be passed, since it's something
     * that would hardly change The values are just provisory and invented
     */
    private static final float ROTATION_LIMIT_DOWN = 0.5f, ROTATION_LIMIT_UP = 0.1f;
    public static final double DEGREES_IN_ONE_TICK = 360 / 42, MOTOR_SPEED = 0.5, DEADBAND = 0.5;
    public double goalRotation, distance;

    public IntakeSubsystem()
    {
        INTAKE_RAISER_MOTOR_RIGHT = new TeamSparkMAX("Intake Motor", Ports.INTAKE_MOTOR_RIGHT);
        INTAKE_RAISER_MOTOR_LEFT = new TeamSparkMAX("Intake Motor", Ports.INTAKE_MOTOR_LEFT);
        INTAKE_TOGGLE_MOTOR = new TeamSparkMAX("Intake Motor", Ports.INTAKE_ON_MOTOR);

        INTAKE_RAISER_MOTOR_LEFT.follow(INTAKE_RAISER_MOTOR_RIGHT);
        INTAKE_RAISER_MOTOR_LEFT.setInverted(true);

        INTAKE_RAISER_MOTOR_LEFT.setSoftLimit(SoftLimitDirection.kForward, ROTATION_LIMIT_DOWN);
        INTAKE_RAISER_MOTOR_RIGHT.setSoftLimit(SoftLimitDirection.kForward, ROTATION_LIMIT_DOWN);

        INTAKE_RAISER_MOTOR_LEFT.setSoftLimit(SoftLimitDirection.kReverse, ROTATION_LIMIT_UP);
        INTAKE_RAISER_MOTOR_RIGHT.setSoftLimit(SoftLimitDirection.kReverse, ROTATION_LIMIT_UP);

        INTAKE_RAISER_MOTOR_LEFT.getEncoder().setPosition(0);
        INTAKE_RAISER_MOTOR_RIGHT.getEncoder().setPosition(0);

        INTAKE_RAISER_MOTOR_LEFT.getEncoder().setPositionConversionFactor(42);
        INTAKE_RAISER_MOTOR_RIGHT.getEncoder().setPositionConversionFactor(42);

    }

    public boolean isStopped()
    {
        return (INTAKE_RAISER_MOTOR_LEFT.get() == 0 && INTAKE_RAISER_MOTOR_RIGHT.get() == 0);
    }

    public double getPosition()
    {
        return INTAKE_RAISER_MOTOR_LEFT.getEncoder().getPosition();
    }

    public void raiseOrLowerIntakeMount(boolean lowered)
    {
        // This is called a ternary operator. Look it up if you don't know what it is.
        INTAKE_RAISER_MOTOR_RIGHT.set(lowered ? -MOTOR_SPEED : MOTOR_SPEED);
        isLowered = lowered;
    }

    // Yes, this is the most descriptive name I could come up with.
    // The original name beore was "toggleIntakeMotors"
    // ...which I then changed to "togleIntakeMotorState"
    // ...which I then changed to this.
    public void turnOnOrStopIntakeMotors(boolean on)
    {
        INTAKE_TOGGLE_MOTOR.set(on ? MOTOR_SPEED : 0);
    }

    @Override
    public void periodic()
    {
        double difference = goalRotation - getCurrentRotation();
        setMotors(
                Math.abs(difference) < DEADBAND ? 0
                        : Math.max(-1,
                                Math.min(distance * Math.sin(difference * Math.PI / distance), 1)),
                "Intake Mount (Deadbanded): Difference: " + difference + ", Distance: " + distance);
    }

    public void setGoalRotation(double degrees)
    {
        distance = degrees - getCurrentRotation();
        goalRotation = degrees;
    }

    public double getCurrentRotation()
    {
        return INTAKE_RAISER_MOTOR_LEFT.getEncoder().getPosition();
    }

    public void setMotors(double power, String reason)
    {
        INTAKE_RAISER_MOTOR_RIGHT.set(Math.max(-1, Math.min(power, 1)) * MOTOR_SPEED, reason);
    }

}
