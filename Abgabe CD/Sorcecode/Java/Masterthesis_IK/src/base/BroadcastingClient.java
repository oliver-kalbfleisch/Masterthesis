package base;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

/*@ author Oliver Kalbfleisch
*
*/
public class BroadcastingClient {
	/**
	 * Method takes in the broadcast messsage and the broadcast address of the network to be used
	 */
	public static void broadcast(String broadcastMessage, InetAddress address) throws IOException {
		try (DatagramSocket socket = new DatagramSocket();) {

			socket.setBroadcast(true);

			byte[] buffer = broadcastMessage.getBytes();

			DatagramPacket packet = new DatagramPacket(buffer, buffer.length, address, 6666);
			System.out.println("sent data!");
			socket.send(packet);
		}

	}
}
