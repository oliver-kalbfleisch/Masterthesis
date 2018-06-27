package base;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import au.edu.federation.utils.Vec2f;
/**
 * This class supplies all needed methods to setu up the UDP Datastream reading Sockets. It also provides all the processing
 * methods needed to retireve the information form the UDP Data and calculate the depth values
 */
public class StereoCalculator {
	private int udpPortLeft;
	private int udpPortRight;

	private Vec2f[] coordinateSetsLeftFiltered;
	private Vec2f[] coordinateSetsRightFiltered;
	private Vec2f[] coordinateSetsLeftUnfiltered;
	private Vec2f[] coordinateSetsRightUnfiltered;

	private float handAngleLeft = 0.0f;
	private float handAngleRight = 0.0f;
	//Multithread reading management
	private static final ReentrantReadWriteLock lockLeft = new ReentrantReadWriteLock(true);
	public volatile static String udpMessageLeft, udpMessageRight;
	private static final ReentrantReadWriteLock lockRight = new ReentrantReadWriteLock(true);

	private static final int CAMERA_IMAGE_WIDTH = 640;
	private static final int CAMERA_IMAGE_HEIGHT = 480;
	//time measurement
	private long epochRight = 0;
	private long epochLeft = 0;
	private long prevEpochRight = 0;
	private long prevEpochLeft = 0;
	
	private int zeroPlaneOffset = 90;
	protected Thread udpthreadLeft;
	protected Thread udpthreadRight;
	//Filtering
	private ValueFilter[] fingerfilters;
	private ValueFilter depthFilter;
	private double frequency = 60; // Hz
	/**
	 * Constructor to initialize a class instance with all needed values
	 */
	public StereoCalculator(int numElements) {
		//UDP Port values must correspond to those set on the sending devices
		this.udpPortLeft = 8888;
		this.udpPortRight = 9999;
		//Init arrrays to store extracted UDO Data
		this.coordinateSetsLeftFiltered = new Vec2f[numElements];
		this.coordinateSetsRightFiltered = new Vec2f[numElements];
		this.coordinateSetsLeftUnfiltered= new Vec2f[numElements];
		this.coordinateSetsRightUnfiltered= new Vec2f[numElements];
		for (int i = 0; i < coordinateSetsLeftFiltered.length; i++) {
			coordinateSetsLeftFiltered[i] = new Vec2f(0, 0);
			coordinateSetsRightFiltered[i] = new Vec2f(0, 0);
			coordinateSetsLeftUnfiltered[i] = new Vec2f(0, 0);
			coordinateSetsRightUnfiltered[i] = new Vec2f(0, 0);
		}
		//Init filters 
		this.fingerfilters = new ValueFilter[numElements];
		this.depthFilter = new ValueFilter(frequency,1, 0);
		for (int i = 0; i < fingerfilters.length; i++) {
			fingerfilters[i] = new ValueFilter(frequency, 1.0, 0.0, 1.0, 0.0);
		}
		// Set filter values for each filter
		// thumb
		fingerfilters[0].setMinCutoffX(0.00001);
		fingerfilters[0].setMinCutoffY(0.00001);
		fingerfilters[0].setBetaX(5.0);
		fingerfilters[0].setBetaY(5.0);
		// index
		fingerfilters[1].setMinCutoffX(0.0001);
		fingerfilters[1].setMinCutoffY(0.00001);
		fingerfilters[1].setBetaX(5.0);
		fingerfilters[1].setBetaY(5.0);
		// middle
		fingerfilters[2].setMinCutoffX(0.00001);
		fingerfilters[2].setMinCutoffY(0.00001);
		fingerfilters[2].setBetaX(0.0001);
		fingerfilters[2].setBetaY(5.0);
		// ring
		fingerfilters[3].setMinCutoffX(0.0001);
		fingerfilters[3].setMinCutoffY(0.0001);
		fingerfilters[3].setBetaX(5.0);
		fingerfilters[3].setBetaY(5.0);
		// litte
		fingerfilters[4].setMinCutoffX(0.00001);
		fingerfilters[4].setMinCutoffY(0.00001);
		fingerfilters[4].setBetaX(5.0);
		fingerfilters[4].setBetaY(1.0);
		// base
		fingerfilters[5].setMinCutoffX(0.000001);
		fingerfilters[5].setMinCutoffY(0.000001);
		fingerfilters[5].setBetaX(5.0);
		fingerfilters[5].setBetaY(5.0);
	}

	/**
	 * 
	 * @author oliver
	 * Inner Class for the UDP Listening threads
	 */
	class UDPProcessThreadLeft implements Runnable {
		private int udpListenPort;
		private boolean running = true;
		private DatagramSocket dsocket;
		private byte[] buffer;
		private DatagramPacket packet;
		private ReentrantReadWriteLock lock;

		public UDPProcessThreadLeft(int udpPort, ReentrantReadWriteLock lock) throws SocketException {
			this.udpListenPort = udpPort;
			this.dsocket = new DatagramSocket(this.udpListenPort);
			this.buffer = new byte[128];
			this.packet = new DatagramPacket(buffer, buffer.length);
			this.lock = lock;
		}

		@Override
		public void run() {
			try {

				while (running) {
					//wait for data to
					dsocket.receive(packet);
					String msg = new String(buffer, 0, packet.getLength());
					try {
						//Threadsafe message extraction
						lock.writeLock().lock();
						udpMessageLeft = msg;
						epochLeft = Long.parseLong(msg.substring(msg.length() - 14, msg.length() - 1));
					} finally {
						lock.writeLock().unlock();
					}
					// Reset the length of the packet before reusing it.
					packet.setLength(buffer.length);
				}

			} catch (

			SocketException e) {
				e.printStackTrace();
			} catch (IOException e) {
				e.printStackTrace();
			} catch (Exception e) {
				e.printStackTrace();
			}

		}

		public void stopListening() {
			this.running = false;
		}

		public void startListening() {
			this.running = true;
		}

	}

	/**
	 * 
	 * @author oliver
	 *
	 */
	class UDPProcessThreadRight implements Runnable {
		private int udpListenPort;
		private boolean running = true;
		private DatagramSocket dsocket;
		private byte[] buffer;
		private DatagramPacket packet;
		private ReentrantReadWriteLock lock;

		public UDPProcessThreadRight(int udpPort, ReentrantReadWriteLock lock) throws SocketException {
			this.udpListenPort = udpPort;
			this.dsocket = new DatagramSocket(this.udpListenPort);
			this.buffer = new byte[128];
			this.packet = new DatagramPacket(buffer, buffer.length);
			this.lock = lock;
		}

		@Override
		public void run() {
			try {

				while (running) {
					dsocket.receive(packet);
					String msg = new String(buffer, 0, packet.getLength());
					try {
						lock.writeLock().lock();
						udpMessageRight = msg;
						epochRight = Long.parseLong(msg.substring(msg.length() - 14, msg.length() - 1));

					} finally {
						lock.writeLock().unlock();
					}
					// Reset the length of the packet before reusing it.
					packet.setLength(buffer.length);
				}

			} catch (

			SocketException e) {
				e.printStackTrace();
			} catch (IOException e) {
				e.printStackTrace();
			} catch (Exception e) {
				e.printStackTrace();
			}

		}

		public void stopListening() {
			this.running = false;
		}

		public void startListening() {
			this.running = true;
		}

	}

	/**
	 * 
	 * @author oliver Inner class holding all needed values for a 1D or a 2D
	 *         filtering
	 */
	class ValueFilter {
		private OneEuroFilter filterX;
		private OneEuroFilter filterY;
		private double filterFrequency;
		private double minCutoffX;
		private double betaX;
		private double minCutoffY;
		private double betaY;

		/**
		 * Constructor for 1D filtering
		 * 
		 * @param filterFrequency
		 * @param minCutoff
		 * @param beta
		 */
		public ValueFilter(double filterFrequency, double minCutoff, double beta) {
			this.betaX = beta;
			this.filterFrequency = filterFrequency;
			this.minCutoffX = minCutoff;
			try {
				this.filterX = new OneEuroFilter(filterFrequency, minCutoff, beta);
			} catch (Exception e) {
				e.printStackTrace();
			}

		}

		/**
		 * Constructor for 2D filtering
		 * 
		 * @param filterFrequency
		 *            -> Frequqncy value in Hz of the incoming data
		 * @param minCutoffX
		 *            -> Cutoff frequqncy vlue in Hz for the x component
		 * @param betaX
		 * @param minCutoffY
		 *            ->Cutoff frequqncy vlue in Hz for the x compone
		 * @param betaY
		 */
		public ValueFilter(double filterFrequency, double minCutoffX, double betaX, double minCutoffY, double betaY) {
			this.betaX = betaX;
			this.betaY = betaY;
			this.filterFrequency = filterFrequency;
			this.minCutoffX = minCutoffX;
			this.minCutoffY = minCutoffY;
			try {
				this.filterX = new OneEuroFilter(filterFrequency, minCutoffX, betaY);
			} catch (Exception e) {
				e.printStackTrace();
			}
			try {
				this.filterY = new OneEuroFilter(filterFrequency, minCutoffY, betaY);
			} catch (Exception e) {
				e.printStackTrace();
			}

		}

		public double getFilterFrequency() {
			return filterFrequency;
		}

		public double getMinCutoff() {
			return minCutoffX;
		}

		public double getBeta() {
			return betaX;
		}

		public void setFilterFrequency(double filterFrequency) {
			this.filterFrequency = filterFrequency;
		}

		public void setMinCutoff(double minCutoff) {
			this.minCutoffX = minCutoff;
		}

		public void setBeta(double beta) {
			this.betaX = beta;
		}

		public double getMinCutoffX() {
			return minCutoffX;
		}

		public double getBetaX() {
			return betaX;
		}

		public double getMinCutoffY() {
			return minCutoffY;
		}

		public double getBetaY() {
			return betaY;
		}

		public void setMinCutoffX(double minCutoffX) {
			this.minCutoffX = minCutoffX;
		}

		public void setBetaX(double betaX) {
			this.betaX = betaX;
		}

		public void setMinCutoffY(double minCutoffY) {
			this.minCutoffY = minCutoffY;
		}

		public void setBetaY(double betaY) {
			this.betaY = betaY;
		}

		public OneEuroFilter getFilterX() {
			return filterX;
		}

		public OneEuroFilter getFilterY() {
			return filterY;
		}

		public void setFilterX(OneEuroFilter filterX) {
			this.filterX = filterX;
		}

		public void setFilterY(OneEuroFilter filterY) {
			this.filterY = filterY;
		}
	}

	/**
	 * Function creates a UDP Port to listen to for data from the camera of the
	 * specified side
	 * 
	 * @param port
	 *            Port on which to listen to UDP Packages
	 * @param cameraSide
	 *            Physical position of the camera in the system (left=0,right=1)
	 * @throws IOException
	 */
	public void createUDPListener(int port, int cameraSide) throws IOException {
		switch (cameraSide) {
		case 0:

			udpthreadLeft = new Thread(new UDPProcessThreadLeft(port, lockLeft));
			udpthreadLeft.start();
			break;
		case 1:

			udpthreadRight = new Thread(new UDPProcessThreadRight(port, lockRight));
			udpthreadRight.start();
			break;
		default:
			System.out.println("invald camera selection parameter " + cameraSide);
			break;
		}
	}

	/**
	 * Function processes the incoming message parts to get the position information
	 * from the input String
	 * 
	 * @param parts
	 *            The split parts of the received UDP String message containing the
	 *            tracking data
	 * @param coordinateSetFiltered
	 *            The 2D vector into which the extracted data will be written
	 */
	private void processMessage(String[] parts, Vec2f[] coordinateSetFiltered, Vec2f[] coordinateSetUnfiltered,float angle) {

		for (int i = 0; i < parts.length - 2; i++) {
			String[] values = parts[i].split(",");
			// Extract value by extracting substrings at correct positions
			int xVal = 0;
			int yVal = 0;
			try {
				xVal = Integer.parseInt(values[0].substring(1));
				yVal = Integer.parseInt(values[1].substring(1, values[1].length() - 1));
				xVal = (int) clamp(0.0, 640.0, (double) (xVal));
				yVal = (int) clamp(0.0, 480.0, (double) (yVal));
			} catch (NumberFormatException e1) {
				e1.printStackTrace();
			}
			
			//save unfiltered values for depth calculation
			 float currentX = (float) -(xVal - (CAMERA_IMAGE_WIDTH / 2.0));
		     float currentY = (float) (yVal - (CAMERA_IMAGE_HEIGHT / 2.0));
			// Filter out jumps in tracking data
			try {
				coordinateSetUnfiltered[i].x = currentX;
				coordinateSetUnfiltered[i].y = currentY;

			} catch (ArrayIndexOutOfBoundsException ae) {
				ae.printStackTrace();
			}
			OneEuroFilter currPosFilterX = fingerfilters[i].getFilterX();
			OneEuroFilter currPosFilterY = fingerfilters[i].getFilterY();
			//Filter values for Visualization
			try {
				currentX = (float) currPosFilterX.filter(-(xVal - (CAMERA_IMAGE_WIDTH / 2.0)));
			} catch (Exception e1) {
				e1.printStackTrace();
			}
			try {
				currentY = (float) currPosFilterY.filter((yVal - (CAMERA_IMAGE_HEIGHT / 2.0)));
			} catch (Exception e) {
				e.printStackTrace();
			}
			try {
				coordinateSetFiltered[i].x = currentX;
				coordinateSetFiltered[i].y = currentY;

			} catch (ArrayIndexOutOfBoundsException ae) {

			}
		}
	}
	private float calculateCameraUpdateFrequency(int side) {
		switch (side) {
		// left
		case 0: {
			long deltaL = epochLeft - prevEpochLeft;
			return ((int) ((1.0f / deltaL) * 1000));
		}
		// right
		case 1: {
			long deltaR = epochRight - prevEpochRight;
			return ((int) ((1.0f / deltaR) * 1000));
		}
		default:
			break;
		}
		return 0.0f;
	}

	/**
	 * Function triggers reading of UDP socket for currently available data for
	 * specified side
	 * 
	 * @param cameraSide
	 * @return
	 */
	public Vec2f[] getDatasetFiltered(int cameraSide) {
		switch (cameraSide) {
		// left camera
		case 0:
			lockLeft.readLock().lock();
			String msgL = udpMessageLeft;
			lockLeft.readLock().unlock();
			String[] partsL = msgL.split(";");
			// Extract hand angle
			handAngleLeft = Float.parseFloat((partsL[partsL.length - 2]));
			// extract other values
			try {
				processMessage(partsL, coordinateSetsLeftFiltered,coordinateSetsLeftUnfiltered, handAngleLeft);
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			return coordinateSetsLeftFiltered;
		// right camera
		case 1:
			lockRight.readLock().lock();
			String msgR = udpMessageRight;
			lockRight.readLock().unlock();
			String[] partsR = msgR.split(";");
			// Extract hand angle
			handAngleRight = Float.parseFloat((partsR[partsR.length - 2]));
			// extract other values
			processMessage(partsR, coordinateSetsRightFiltered,coordinateSetsRightUnfiltered, handAngleRight);
			return coordinateSetsRightFiltered;
		default:
			System.out.println("invalid camera input parameter " + cameraSide);
			return new Vec2f[0];
		}

	}
	public Vec2f[] getDatasetUnfiltered(int cameraSide)
	{
		switch (cameraSide) {
		// left camera
		case 0:
			return coordinateSetsLeftUnfiltered;
		// right camera
		case 1:
			return coordinateSetsRightUnfiltered;
		default:
			System.out.println("invalid camera input parameter " + cameraSide);
			return new Vec2f[0];
		}
	}

	/*
	 * Raspberry Pi 2 camera Params camera Dist= 50–75 mm for human vision viewing
	 * angle in degree v2 module x:62.2 y:48.8
	 */
	/**
	 * 
	 * @param rightX
	 * @param leftX
	 * @return
	 */
	public float calculateZDistance(float rightX, float leftX)

	{
		// compensated Calculation formula Z=k*d^z
		double k = 4543.320129238;
		double z = -1.0354229928;
		double disparity =rightX - leftX;
		try {
			disparity = depthFilter.getFilterX().filter(disparity);
		} catch (Exception e) {
			e.printStackTrace();
		}
		double calculatedZDistance = k * (1.0 / (Math.pow(Math.abs(disparity), -z)));
		// Clamp depth ranges to known values of the tracking space
		calculatedZDistance = clamp(0.0, 90.0, calculatedZDistance);
		// Coarse value Filtering
		if (!Double.isNaN(calculatedZDistance)) {
			//System.out.println(-calculatedZDistance + zeroPlaneOffset);
			return (float) (-calculatedZDistance + zeroPlaneOffset);
		}
		return -1.0f;
	}

	private double clamp(double min, double max, double value) {
		return Math.max(min, Math.min(value, max));

	}

	public int getUDPPortLeft() {
		return udpPortLeft;
	}

	public void setUDPPortLeft(int uDPPortLeft) {
		udpPortLeft = uDPPortLeft;
	}

	public int getUDPPortRight() {
		return udpPortRight;
	}

	public void setUDPPortRight(int uDPPortRight) {
		udpPortRight = uDPPortRight;
	}
	public Vec2f[] getCoordinateSetsLeft() {
		return coordinateSetsLeftFiltered;

	}

	public Vec2f[] getCoordinateSetsRight() {
		return coordinateSetsRightFiltered;
	}

	public long getEpochRight() {
		return epochRight;
	}

	public long getEpochLeft() {
		return epochLeft;
	}

	public float getHandAngleLeft() {
		return handAngleLeft;
	}

	public float getHandAngleRight() {
		return handAngleRight;
	}

}
