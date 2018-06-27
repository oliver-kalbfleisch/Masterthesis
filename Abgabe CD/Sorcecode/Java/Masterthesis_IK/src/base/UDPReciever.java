package base;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.time.Instant;
/*@author Oliver Kalbfleisch
*
*/
public class UDPReciever {
	
/*public static void main(String[] args) {
	UDPReciever reciever= new UDPReciever();
	reciever.setup("192.168.1.255",8888);	
}	
*/	
	
public void setup(String senderAdress, int port)
{
	//try to setup datagram socket
	try {
		//Create Socket for listening to specified port
		System.out.println("Setting up Soket");
		DatagramSocket dSocket= new DatagramSocket(port);
		//Create buffer for recieved Bytes, excess bytes will be discarded!!
		System.out.println("creating Buffer");
		byte[] buffer= new byte[128];
		//Packet to but data from bugffer into
		System.out.println("creating Datapacket");
		DatagramPacket dPacket= new DatagramPacket(buffer, buffer.length);
		System.out.println("Listening:...");
		while(true)
		{
			dSocket.receive(dPacket);
			System.out.println(dPacket.getLength());
			String msg= new String(buffer,0,dPacket.getLength());
			System.out.println(dPacket.getAddress().getHostName()+":"+msg);
			msg="";
			 // Reset the length of the packet before reusing it.
	        dPacket.setLength(buffer.length);
		}
		
		
	} catch (IOException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
	
	
}
}
