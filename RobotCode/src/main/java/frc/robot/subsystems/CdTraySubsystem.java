package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.lib.core.motors.TeamSparkMAX;

public class CdTraySubsystem extends SubsystemBase
{
	// guys i think i'm starting to get the hang of thi- oh nevermind
	private Compressor mainCompressor;

	private DoubleSolenoid cdArmLeft, cdArmRight;

	public TeamSparkMAX intakeMotor;

	public boolean closed;

	public CdTraySubsystem()
	{
		mainCompressor = new Compressor(Ports.PNEUMATICS_HUB, PneumaticsModuleType.REVPH);
		cdArmRight = new DoubleSolenoid(Ports.PNEUMATICS_HUB, PneumaticsModuleType.REVPH, 0, 0);
		cdArmLeft = new DoubleSolenoid(Ports.PNEUMATICS_HUB, PneumaticsModuleType.REVPH, 0, 0);

		mainCompressor.enableDigital();

		cdArmLeft.set(Value.kReverse);
		cdArmRight.set(Value.kReverse);
		// rip comedy, you will not be forgotten

	public DoubleSolenoid getCdArmLeft()
	{
		return cdArmLeft;
	}

	public boolean getClosed()
	{
		return cdArmLeft.get() == Value.kForward && cdArmRight.get() == Value.kForward;
	}

	public void setSolenoid(Value mode)
	{
		cdArmLeft.set(mode);
		cdArmRight.set(mode);
		System.out.println("Setting solenoids to " + mode);
	}

}
