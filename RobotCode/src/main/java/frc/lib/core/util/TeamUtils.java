package frc.lib.core.util;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.HashMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import frc.lib.core.Scalar;

public class TeamUtils
{
	public static double getCurrentTime()
	{
		return 1.0 * System.nanoTime() / 1e9;
		// Why nanoseconds????????
	}

	public static void sendToNetworkTable(String tableName, String key, Object value)
	{
		NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);
		if (table == null)
		{
			return;
		}
		NetworkTableEntry entry = table.getEntry(key);
		if (entry == null)
		{
			return;
		}
		entry.setValue(value);
	}

	public static Object getFromNetworkTable(String tableName, String key)
	{
		NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);
		if (table == null)
		{
			return null;
		}
		NetworkTableEntry entry = table.getEntry(key);
		if (entry == null)
		{
			return null;
		}
		NetworkTableValue value = entry.getValue();
		if (value == null)
		{
			return null;
		}
		return value.getValue();
	}

	public static boolean checkTolerance(double currentValue, double targetValue, double epsilon)
	{
		if (Math.abs(currentValue - targetValue) <= epsilon)
			return true;
		else
		{
			return false;
		}
	}

	public static double map(double value, double start1, double end1, double start2, double end2)
	{

		if (Math.abs(end1 - start1) < Scalar.EPSILON)
		{
			throw new ArithmeticException("/ 0");
		}

		double offset = start2;
		double ratio = (end2 - start2) / (end1 - start1);
		return ratio * (value - start1) + offset;
	}

	public static HashMap<Double, Double> CsvReader(String filepath)
	{
		HashMap<Double, Double> data = new HashMap<Double, Double>();
		try
		{
			BufferedReader br = new BufferedReader(new FileReader(filepath));
			String line;
			while ((line = br.readLine()) != null)
			{
				if (line.charAt(0) == '#' || line.isEmpty())
				{
					continue;
				}
				String[] array = line.split(",", 2);
				data.put(Double.parseDouble(array[0]), Double.parseDouble(array[1]));
			}
		}
		catch (Exception e)
		{
			System.out.println("Failed to load file (file name is prob wrong)");
		}
		return data;
	}

	static public int LowerBound(double n, Double[] array)
	{
		int l = -1;
		int r = array.length;
		while (l + 1 < r)
		{
			int m = (l + r) >>> 1;
			if (array[m] >= n)
			{
				r = m;
			}
			else
			{
				l = m;
			}
		}
		return r;
	};

	static public int UpperBound(double n, Double[] array)
	{
		int l = -1;
		int r = array.length;
		while (l + 1 < r)
		{
			int m = (l + r) >>> 1;
			if (array[m] <= n)
			{
				l = m;
			}
			else
			{
				r = m;
			}
		}
		return l + 1;
	};

}