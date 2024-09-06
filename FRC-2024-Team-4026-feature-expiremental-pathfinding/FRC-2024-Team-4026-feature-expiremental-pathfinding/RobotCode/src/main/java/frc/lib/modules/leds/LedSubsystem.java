package frc.lib.modules.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedSubsystem extends SubsystemBase {

	/** An LED strip */
	private AddressableLED led;
	/** A buffer with data on the LED strip states */
	private AddressableLEDBuffer buffer;

	/** Number of pixels on the LED strip */
	private int length;

	/** The current desired color of the LED strip */
	private TeamColor currentColor;
	/** The color to turn off the LED strip */
	private final TeamColor OFF_COLOR;

	/** The number of remaining times to flash the LED strip */
	private int numFlashes;

	private int periodsPassed;

	/** Indicates whether the LEDs are off when flashing */
	private boolean off;

	public LedSubsystem() {
		this.length = LedConstants.LENGTH;

		// Configure the LED buffer
		buffer = new AddressableLEDBuffer(length);

		// Configure the LED strip
		led = new AddressableLED(Ports.ADDRESSABLE_LED);
		led.setLength(length);
		led.setData(buffer);
		led.start();

		numFlashes = 0;
		periodsPassed = 0;

		off = true;
		OFF_COLOR = LedConstants.OFF_COLOR;
		currentColor = OFF_COLOR;
	}

	/** 
	 * Sets the LED strip to a color
	 * 
	 * @param color the desired color for the LED strip
	 */
	public void setAllPixels(TeamColor color) {
		currentColor = color;

		for (int i = 0; i < length; i++)
			buffer.setRGB(i, color.r, color.g, color.b);
		
		this.updateData();
	}

	/** 
	 * Sets the LED strip to a color and flashes it on and off a specified number of times
	 * 
	 * @param color the desired color for the LED strip
	 * @param numFlashes the number of times to flash the LED strip
	 */
	public void flashAllPixels(TeamColor color, int numFlashes) {
		setAllPixels(color);

		this.numFlashes = numFlashes;
	}

	@Override
	public void periodic() {
		periodsPassed++;

		// Toggle the LED strip every 100 milliseconds if it should be flashing
		if (periodsPassed == 5 && numFlashes > 0) {
			// Toggle the LED strip on
			if (off) {
				for (int i = 0; i < length; i++)
					buffer.setRGB(i, currentColor.r, currentColor.g, currentColor.b);
		
				this.updateData();

				off = false;
			} 
			// Toggle the LED strip off
			else {
				for (int i = 0; i < length; i++)
					buffer.setRGB(i, OFF_COLOR.r, OFF_COLOR.g, OFF_COLOR.b);
		
				this.updateData();

				off = true;
				numFlashes--;
			}
		}

		// Reset periods passed every 100 milliseconds
		if (periodsPassed > 5) {
			periodsPassed = 0;
		}
	}

	/** Set the state of the LEDs to match the buffer */
	public void updateData() {
		led.setData(buffer);
	}

	/** @return the length of the LED strip */
	public int getLength() {
		return length;
	}

	/** @return the current color of the LED buffer */
	public TeamColor getCurrentColor() {
		return currentColor;
	}

}
