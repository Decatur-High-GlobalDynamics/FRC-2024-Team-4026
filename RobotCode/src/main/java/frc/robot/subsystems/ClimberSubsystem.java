package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.core.motors.TeamTalonFX;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.Ports;
import frc.lib.core.ILogSource;

public class ClimberSubsystem extends SubsystemBase implements ILogSource
{

    private TeamTalonFX extendMotorLeft;
    private TeamTalonFX extendMotorRight;

    public ClimberSubsystem()
    {
        // sets extension of left and right motors to given extension length
        extendMotorLeft = new TeamTalonFX("Subsystems.Climber.ExtendRight",
                Ports.CLIMBER_EXTEND_RIGHT_MOTOR);
        extendMotorRight = new TeamTalonFX("Subsystems.Climber.ExtendLeft",
                Ports.CLIMBER_EXTEND_LEFT_MOTOR);
        extendMotorLeft.setNeutralMode(NeutralMode.Brake);
        extendMotorRight.setNeutralMode(NeutralMode.Brake);
    }

    public void setPowers(double leftPower, double rightPower, String reason)
    {
        if (!(leftMotorPowerCheck(leftPower)))
        {
            leftPower = 0;
        }
        if (!(rightMotorPowerCheck(rightPower)))
        {
            rightPower = 0;
        }
        if (leftPower != 0)
            leftPower /= 3;
        if (rightPower != 0)
            rightPower /= 3;

        extendMotorLeft.set(-leftPower / 3);
        extendMotorLeft.set(-rightPower / 3);
        logFinest("Set  left climber motors to " + -leftPower / 3);
        logFinest("Set right climber motors to" + -leftPower / 3);
    }

    // checks if the power level is too high or low for both motors.
    public boolean leftMotorPowerCheck(double power)
    {
        return (extendMotorLeft.getCurrentEncoderValue() > ClimberConstants.MAX_EXTENSION_LEFT
                && power >= 0)
                || (extendMotorLeft.getCurrentEncoderValue() < ClimberConstants.MIN_EXTENSION_LEFT
                        && power <= 0);
    }

    public boolean rightMotorPowerCheck(double power)
    {
        return (extendMotorRight.getCurrentEncoderValue() > ClimberConstants.MAX_EXTENSION_RIGHT
                && power <= 0)
                || (extendMotorRight.getCurrentEncoderValue() < ClimberConstants.MIN_EXTENSION_RIGHT
                        && power >= 0);
    }

}