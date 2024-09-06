package frc.lib.modules.leds;

public class TeamColor
{

	// Preset colors
	public static final TeamColor Red = new TeamColor(255, 0, 0);
	public static final TeamColor Green = new TeamColor(0, 255, 0);
	public static final TeamColor Blue = new TeamColor(0, 0, 200);
	public static final TeamColor Yellow = new TeamColor(255, 127, 0);
	public static final TeamColor Cyan = new TeamColor(0, 255, 255);
	public static final TeamColor Magenta = new TeamColor(255, 0, 255);
	public static final TeamColor White = new TeamColor(255, 255, 255);
	public static final TeamColor Black = new TeamColor(0, 0, 0);

	public boolean hsv;

	public int r, g, b;
	public int h, s, v;
	// It is ultimately pointless to make these private.

	public TeamColor(int red, int green, int blue)
	{
		this(red, green, blue, false);
	}

	public TeamColor(int[] color)
	{
		this(color[0], color[1], color[2], false);
	}

	public static TeamColor fromHSV(int h, int s, int v)
	{
		return new TeamColor(h, s, v, true);
	}

	private TeamColor(int red, int green, int blue, boolean hsv)
	{
		this.hsv = hsv;
		if (!hsv)
		{
			this.r = red;
			this.g = green;
			this.b = blue;
		}
		else
		{
			this.h = red;
			this.s = green;
			this.v = blue;
		}
	}

	public int[] getList()
	{
		return hsv ? new int[]
		{
				h, s, v
		} : new int[]
		{
				r, g, b
		};
	}

	public int get(int index)
	{
		switch (index)
		{
		case 0:
			return hsv ? h : r;
		case 1:
			return hsv ? s : g;
		case 2:
			return hsv ? v : b;
		}
		return 0; // If index > 2, return 0
	}

	public TeamColor clone()
	{
		return this.hsv ? new TeamColor(this.r, this.g, this.b, true)
				: new TeamColor(this.h, this.s, this.b, false);
	}

}
