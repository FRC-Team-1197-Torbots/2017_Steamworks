package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

public class TorLidar
{
	int lastDistance = 0;
	SerialPort m_port;

	/** 
	 * Use 9600 baud rate and SerialPort.Port.kOnboard to instantiate.
	 *  port = new SerialPort(9600, SerialPort.Port.kOnboard);
	 *  Make sure to turn off console out on the roborio
	 */
	public TorLidar(SerialPort sp)
	{
		m_port = sp;
	}

	public int getDistance()
	{
		m_port.writeString("r\n");
		Timer.delay(0.05D);

		String distance = m_port.readString();
		if (distance.length() == 0) {
			return lastDistance;
		}

		int start = 0;
		char[] dist = distance.toCharArray();
		for (int i = 0; i < distance.length(); i++) {
			if (dist[i] != '0')
			{
				start = i;
				break;
			}
		}
		int end = 0;
		for (int i = start; i < distance.length(); i++) {
			if (dist[i] == '\n')
			{
				end = i - 1;
				break;
			}
		}
		distance = distance.substring(start, end);

		Integer intDist = new Integer(distance);
		lastDistance = intDist;

		return intDist.intValue();
	}
}

