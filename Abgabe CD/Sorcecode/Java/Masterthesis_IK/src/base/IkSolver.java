package base;

import static org.lwjgl.glfw.GLFW.GLFW_FALSE;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_A;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_C;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_D;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_E;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_ESCAPE;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_F;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_Q;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_R;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_S;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_W;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_H;
import static org.lwjgl.glfw.GLFW.GLFW_RESIZABLE;
import static org.lwjgl.glfw.GLFW.GLFW_TRUE;
import static org.lwjgl.glfw.GLFW.GLFW_VISIBLE;
import static org.lwjgl.glfw.GLFW.glfwCreateWindow;
import static org.lwjgl.glfw.GLFW.glfwDefaultWindowHints;
import static org.lwjgl.glfw.GLFW.glfwDestroyWindow;
import static org.lwjgl.glfw.GLFW.glfwGetPrimaryMonitor;
import static org.lwjgl.glfw.GLFW.glfwGetTime;
import static org.lwjgl.glfw.GLFW.glfwGetVersionString;
import static org.lwjgl.glfw.GLFW.glfwGetVideoMode;
import static org.lwjgl.glfw.GLFW.glfwGetWindowSize;
import static org.lwjgl.glfw.GLFW.glfwInit;
import static org.lwjgl.glfw.GLFW.glfwMakeContextCurrent;
import static org.lwjgl.glfw.GLFW.glfwPollEvents;
import static org.lwjgl.glfw.GLFW.glfwSetKeyCallback;
import static org.lwjgl.glfw.GLFW.glfwSetWindowPos;
import static org.lwjgl.glfw.GLFW.glfwSetWindowShouldClose;
import static org.lwjgl.glfw.GLFW.glfwShowWindow;
import static org.lwjgl.glfw.GLFW.glfwSwapBuffers;
import static org.lwjgl.glfw.GLFW.glfwSwapInterval;
import static org.lwjgl.glfw.GLFW.glfwTerminate;
import static org.lwjgl.glfw.GLFW.glfwWindowHint;
import static org.lwjgl.glfw.GLFW.glfwWindowShouldClose;
import static org.lwjgl.opengl.GL11.GL_COLOR_BUFFER_BIT;
import static org.lwjgl.opengl.GL11.GL_DEPTH_BUFFER_BIT;
import static org.lwjgl.opengl.GL11.glClear;
import static org.lwjgl.opengl.GL11.glClearColor;
import static org.lwjgl.system.MemoryStack.stackPush;
import static org.lwjgl.system.MemoryUtil.NULL;

import java.io.IOException;
import java.net.InetAddress;
import java.net.SocketException;
import java.nio.IntBuffer;
import java.util.concurrent.TimeUnit;

import org.joml.Vector3f;
import org.lwjgl.glfw.GLFWCursorPosCallback;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.glfw.GLFWKeyCallback;
import org.lwjgl.glfw.GLFWVidMode;
import org.lwjgl.opengl.GL;
import org.lwjgl.openvr.VR;
import org.lwjgl.system.MemoryStack;
import org.w3c.dom.ElementTraversal;

import Input.KeyboardHandler;
import au.edu.federation.caliko.BoneConnectionPoint;
import au.edu.federation.caliko.FabrikBone3D;
import au.edu.federation.caliko.FabrikChain3D;
import au.edu.federation.caliko.FabrikChain3D.BaseboneConstraintType3D;
import au.edu.federation.caliko.FabrikJoint3D.JointType;
import au.edu.federation.caliko.FabrikStructure3D;
import au.edu.federation.caliko.visualisation.Axis;
import au.edu.federation.caliko.visualisation.Camera;
import au.edu.federation.caliko.visualisation.FabrikConstraint3D;
import au.edu.federation.caliko.visualisation.FabrikModel3D;
import au.edu.federation.caliko.visualisation.Grid;
import au.edu.federation.caliko.visualisation.Point3D;
import au.edu.federation.utils.Colour4f;
import au.edu.federation.utils.Mat4f;
import au.edu.federation.utils.Utils;
import au.edu.federation.utils.Vec2f;
import au.edu.federation.utils.Vec3f;
import OpenVR.*;

public class IkSolver {

	private static final String BROADCASTIP = "192.168.1.255";
	// Define cardinal axes
	static final Vec3f X_AXIS = new Vec3f(1.0f, 0.0f, 0.0f);
	static final Vec3f Y_AXIS = new Vec3f(0.0f, 1.0f, 0.0f);
	static final Vec3f Z_AXIS = new Vec3f(0.0f, 0.0f, 1.0f);
	// This prevents our window from crashing later on.
	private GLFWErrorCallback errorCB;
	private GLFWKeyCallback keyCB;
	private GLFWCursorPosCallback cpCallback;
	private FabrikChain3D[] fingers = new FabrikChain3D[5];
	private static Colour4f[] cols = new Colour4f[5];
	static final int WIDTH = 640;
	static final int HEIGHT = 480; // Window width and height
	private HandData handData = new HandData(6);
	private Axis axis;
	private Axis structureAxis;
	private Grid baseGrid;
	private OBJModel3D model;
	private OBJModel3D trackingObjModel;
	private long window; // Window handle
	private Camera camera;
	/** time at last frame */
	long lastFrame;

	/** frames per second */
	int fps;
	/** last fps time */
	long lastFPS;

	private Mat4f projectionMatrix = Mat4f.createPerspectiveProjectionMatrix(60.0f, (float) WIDTH / (float) HEIGHT,
			1.0f, 10000.0f);
	private Mat4f projectionViewMatrix;
	private Mat4f viewMatrix = new Mat4f();
	private FabrikStructure3D handStructureModel = new FabrikStructure3D();
	private float lenghtMultiplier = 0.75f;
	private float depthOffset = -0.0f;
	private Vec3f boneDirection = new Vec3f(0.0f, 1.0f, .0f);
	private Vec3f handBasePos;
	private double pixelConversionFactorX;
	private double pixelConversionFactorY;

	// Initialize fingers
	// Thumb
	private FabrikChain3D thumb = new FabrikChain3D();
	private FabrikChain3D thumbConnectorChain = new FabrikChain3D();
	// index finger
	private FabrikChain3D indexF = new FabrikChain3D();
	private Colour4f blue = new Colour4f(Utils.BLUE);
	// middle Finger
	private FabrikChain3D middleF = new FabrikChain3D();
	private Colour4f yellow = new Colour4f(Utils.YELLOW);
	// ring Finger
	private FabrikChain3D ringF = new FabrikChain3D();
	private Colour4f cyan = new Colour4f(Utils.CYAN);
	// little Finger
	private FabrikChain3D littleF = new FabrikChain3D();
	private Colour4f magenta = new Colour4f(Utils.MAGENTA);
	private FabrikBone3D handBase;
	private FabrikChain3D handBaseChain = new FabrikChain3D();
	// bone for object representation
	private FabrikBone3D trackingObjBone;
	private boolean trackerCalibrated = false;
	private Vec3f objectCalibrationPos = new Vec3f();

	// STEREO STUFF
	private StereoCalculator calc;

	// Tracking stuff
	private TrackedObjectComponent htcVivetracker;

	public void run() throws SocketException, InterruptedException { // Create our chain

		try {
			init();
			loop();
			// Release window and window callbacks
			glfwDestroyWindow(window);
			keyCB.free();
			cpCallback.free();
		} finally {
			// Terminate GLFW and release the GLFWErrorCallback
			glfwTerminate();
			errorCB.free();
		}
	}

	public void handleCameraMovement(int key, int action) {
		camera.handleKeypress(key, action);
	}

	private void init() throws InterruptedException {
		//Init HTC tracker system
		try {
			SimpleOpenVRWrapper.initOpenVR();
		} catch (Exception e2) {
			System.err.println("init openvr wrapper failed");
			e2.printStackTrace();
		}
		try {
			htcVivetracker = new TrackedObjectComponent(VR.ETrackedDeviceClass_TrackedDeviceClass_GenericTracker);
		} catch (Exception e2) {
			System.err.println("init of tracker failed");
			e2.printStackTrace();
		}
		Colour4f baseColor = new Colour4f(Utils.BLACK);

		Vec3f handBaseBoneEnd = setupHandBase(baseColor);

		try {
			setupThumb(handBaseBoneEnd, baseColor);
		} catch (Exception e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

		try {
			setupIndexFinger(handBaseBoneEnd, baseColor);
		} catch (Exception e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

		try {
			setupMiddleFinger(handBaseBoneEnd, baseColor);
		} catch (Exception e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

		try {
			setupRingFinger(handBaseBoneEnd, baseColor);
		} catch (Exception e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

		try {
			setupLittleFinger(handBaseBoneEnd, baseColor);
		} catch (Exception e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		try {
			setupTrackedObject();
		} catch (Exception e) {
			// TODO: handle exception
		}

		// calculate the real world distance one pixel covers at max depth
		pixelConversionFactorX = (Math.tan(Math.toRadians(31.1f)) * 90) / 320.0f;
		pixelConversionFactorY = (Math.tan(Math.toRadians(24.4f)) * 90) / 240.0f;
		// Setup matrices for camera
		viewMatrix.setIdentity();
		Mat4f rotMat = new Mat4f();
		rotMat.setIdentity();
		rotMat = rotMat.rotateAboutLocalAxisDegs(90, Y_AXIS);
		viewMatrix = viewMatrix.translate(0.0f, 0.0f, -90.0f);
		projectionViewMatrix = projectionMatrix.times(viewMatrix);

		// Init of UDP listening and data Calculation in separate thread
		int numColors = 6;
		try {
			setupStereoCalculator(numColors);
		} catch (Exception e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		glfwSetup();
		try {
			GL.createCapabilities();
			axis = new Axis(1000.0f, 2.0f);
			baseGrid = new Grid(100.0f, 100.0f, 0.1f, 100);
			// structureAxis = new Axis(5.0f, 1.0f);

			try {
				model = new OBJModel3D("cylinder_2.obj", 1.0f);
				trackingObjModel = new OBJModel3D("lowpoly_torous.obj", 1.0f);
			} catch (Exception e) {
				e.printStackTrace();
				System.exit(-1);
			}
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.exit(0);
		}
		TimeUnit.SECONDS.sleep(5);
		try {
			// Use loacal network broadcast address!!
			BroadcastingClient.broadcast("master", InetAddress.getByName(BROADCASTIP));
		} catch (Exception e) {
			System.out.println("Could not send start signal to network, check Connection");
			e.printStackTrace();
		}

	}

	private void glfwSetup() {
		// Setup an error callback. The default implementation // will print the error
		// message in System.err.
		GLFWErrorCallback.createPrint(System.err).set();
		// Initialize GLFW. Most GLFW functions will not work before doing this.
		if (!glfwInit())
			throw new IllegalStateException("Unable to initialize GLFW");
		// Configure GLFW
		glfwDefaultWindowHints(); // optional, the current window hints are already the default
		glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE); // the window will stay hidden after creation
		glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE); // the window will be resizable
		// Create the window
		window = glfwCreateWindow(WIDTH, HEIGHT, "IkSolver Visual Demo", NULL, NULL);
		if (window == NULL)
			throw new RuntimeException("Failed to create the GLFW window");

		// Setup a key callback
		glfwSetKeyCallback(window, keyCB = new KeyboardHandler());
		// Get the thread stack and push a new frame
		try (MemoryStack stack = stackPush()) {
			IntBuffer pWidth = stack.mallocInt(1); // int*
			IntBuffer pHeight = stack.mallocInt(1); // int*

			// Get the window size passed to glfwCreateWindow
			glfwGetWindowSize(window, pWidth, pHeight);

			// Get the resolution of the primary monitor
			GLFWVidMode vidmode = glfwGetVideoMode(glfwGetPrimaryMonitor());

			// Center the window
			glfwSetWindowPos(window, (vidmode.width() - pWidth.get(0)) / 2, (vidmode.height() - pHeight.get(0)) / 2);
		} // the stack frame is popped automatically
			// Make the OpenGL context current
		glfwMakeContextCurrent(window);
		// Enable v-sync
		glfwSwapInterval(1);
		// Make the window visible
		glfwShowWindow(window);
	}

	private void setupTrackedObject() {
		Vec3f objBase = new Vec3f(10.0f, 0.0f, 0.0f);
		Vec3f objEnd = new Vec3f(10.0f, 0.0f, 1.0f);
		trackingObjBone = new FabrikBone3D(objBase, objEnd);
	}

	/**
	 * @param depthOffset
	 * @param baseColor
	 * @return
	 */
	private Vec3f setupHandBase(Colour4f baseColor) {
		handBasePos = new Vec3f(0.0f, 0.0f, depthOffset);
		Vec3f handBaseBoneEnd = new Vec3f(handBasePos.x, handBasePos.y + 0.01f * lenghtMultiplier, handBasePos.z);
		handBase = new FabrikBone3D(handBasePos, handBaseBoneEnd);
		handBaseChain.addBone(handBase);
		handBaseChain.setFixedBaseMode(true);
		handStructureModel.addChain(handBaseChain);
		handBase.setColour(baseColor.lighten(0.4f));
		handBaseChain.setFixedBaseMode(true);
		return handBaseBoneEnd;
	}

	/**
	 * @param lenghtMultplier
	 * @param boneDirection
	 * @param handBaseBoneEnd
	 * @param baseColor
	 */
	private void setupThumb(Vec3f handBaseBoneEnd, Colour4f baseColor) {
		// Thumb------------------------------------------------------------
		Vec3f thumbConEnd = new Vec3f(-4.0f * lenghtMultiplier, 0.0f * lenghtMultiplier, 0.0f);
		FabrikBone3D thumbConnect = new FabrikBone3D(handBaseBoneEnd, thumbConEnd);
		FabrikChain3D thumbConChain = new FabrikChain3D();
		thumbConnect.setColour(baseColor);
		thumbConChain.addBone(thumbConnect);
		thumbConChain.setFixedBaseMode(true);
		handStructureModel.addChain(thumbConChain);
		Vec3f thumbBaseStart = new Vec3f();
		Vec3f thumbBaseEnd = thumbBaseStart.plus(thumbConnect.getDirectionUV().times(4.5f * lenghtMultiplier));
		// thumb MP
		FabrikBone3D thumbBase = new FabrikBone3D(thumbBaseStart, thumbBaseEnd);
		cols[0] = new Colour4f(Utils.RED);
		thumbBase.setColour(cols[0].lighten(0.4f));
		thumb.setMaxIterationAttempts(10);
		thumb.addBone(thumbBase);
		// Thumb MP
		thumb.addConsecutiveHingedBone(boneDirection, 3.5f * lenghtMultiplier, JointType.LOCAL_HINGE, X_AXIS, 0, 95,
				Y_AXIS, cols[0].darken(0.4f));
		// thumb PIP
		thumb.addConsecutiveHingedBone(boneDirection, 3.0f * lenghtMultiplier, JointType.LOCAL_HINGE, X_AXIS, 0, 95,
				Y_AXIS, cols[0].darken(0.4f));
		fingers[0] = thumb;
		thumb.setFixedBaseMode(true);
		Vec3f constraintAxis = Vec3f.rotateZDegs(boneDirection, 75.0f);
		thumb.setRotorBaseboneConstraint(BaseboneConstraintType3D.LOCAL_ROTOR, constraintAxis, 45.0f);
		handStructureModel.connectChain(thumb, 1, 0, BoneConnectionPoint.END);
	}

	/**
	 * @param numColors
	 */
	private void setupStereoCalculator(int numColors) {
		calc = new StereoCalculator(numColors);
		// // // Step one init UPD Listeners for both cameras
		try {
			calc.createUDPListener(8888, 0);
		} catch (IOException e) {
			e.printStackTrace();
		}
		try {
			calc.createUDPListener(9999, 1);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	/**
	 * @param lenghtMultplier
	 * @param boneDirection
	 * @param handBaseBoneEnd
	 * @param baseColor
	 */
	private void setupIndexFinger(Vec3f handBaseBoneEnd, Colour4f baseColor) {
		/*
		 *
		 */
		// Index Finger----------------------------------------------------
		Vec3f indexConEnd = new Vec3f(-3.0f * lenghtMultiplier, 4.0f * lenghtMultiplier, 0.0f);
		Vec3f indexFBaseStart = new Vec3f();
		Vec3f indexFBaseEnd = indexFBaseStart.plus(boneDirection.times(5.0f * lenghtMultiplier));
		FabrikBone3D indexConnect = new FabrikBone3D(handBaseBoneEnd, indexConEnd);
		indexConnect.setColour(baseColor);
		FabrikChain3D indexConChain = new FabrikChain3D();
		indexConChain.addBone(indexConnect);
		indexConChain.setFixedBaseMode(true);
		handStructureModel.addChain(indexConChain);
		//
		// index MP
		FabrikBone3D indexFBase = new FabrikBone3D(indexFBaseStart, indexFBaseEnd);
		cols[1] = new Colour4f(Utils.BLUE);
		indexFBase.setColour(cols[1].darken(0.4f));
		indexF.setMaxIterationAttempts(10);
		indexF.addBone(indexFBase);
		// index PIP
		indexF.addConsecutiveHingedBone(boneDirection, 3.0f * lenghtMultiplier, JointType.LOCAL_HINGE, X_AXIS, 0, 110,
				Y_AXIS, cols[1].lighten(0.4f));
		// index DIP
		indexF.addConsecutiveHingedBone(boneDirection, 2.0f * lenghtMultiplier, JointType.LOCAL_HINGE, X_AXIS, 0, 90,
				Y_AXIS, cols[1].darken(0.4f));
		fingers[1] = indexF;
		indexF.setFixedBaseMode(true);
		Vec3f constraintAxis = Vec3f.rotateZDegs(boneDirection, 15.0f);
		indexF.setRotorBaseboneConstraint(BaseboneConstraintType3D.LOCAL_ROTOR, constraintAxis, 20.0f);
		handStructureModel.connectChain(indexF, 3, 0, BoneConnectionPoint.END);
	}

	/**
	 * @param lenghtMultplier
	 * @param boneDirection
	 * @param handBaseBoneEnd
	 * @param baseColor
	 */
	private void setupLittleFinger(Vec3f handBaseBoneEnd, Colour4f baseColor) {
		// little Finger
		Vec3f littleFConStart = handBaseBoneEnd;
		Vec3f littlefConEnd = new Vec3f(4.5f * lenghtMultiplier, 4.0f * lenghtMultiplier, 0.0f);
		Vec3f littleFBaseStart = new Vec3f();
		Vec3f littleFBaseEnd = littleFBaseStart.plus(boneDirection.times(4.0f * lenghtMultiplier));
		FabrikBone3D littleFConnect = new FabrikBone3D(littleFConStart, littlefConEnd);
		littleFConnect.setColour(baseColor);
		FabrikChain3D littleFConChain = new FabrikChain3D();
		littleFConChain.addBone(littleFConnect);
		handStructureModel.addChain(littleFConChain);

		// little MP
		FabrikBone3D littleFBase = new FabrikBone3D(littleFBaseStart, littleFBaseEnd);
		cols[4] = new Colour4f(Utils.MAGENTA);
		littleFBase.setColour(cols[4].darken(0.4f));
		littleF.setMaxIterationAttempts(10);
		littleF.addBone(littleFBase);

		// little PIP
		littleF.addConsecutiveHingedBone(boneDirection, 2.0f * lenghtMultiplier, JointType.LOCAL_HINGE, X_AXIS, 0, 95,
				Y_AXIS, cols[4].lighten(0.4f));
		// little DIP
		littleF.addConsecutiveHingedBone(boneDirection, 2.0f * lenghtMultiplier, JointType.LOCAL_HINGE, X_AXIS, 0, 95,
				Y_AXIS, cols[4].darken(0.4f));
		fingers[4] = littleF;
		Vec3f constraintAxis = Vec3f.rotateZDegs(boneDirection, -25.0f);
		littleF.setRotorBaseboneConstraint(BaseboneConstraintType3D.LOCAL_ROTOR, constraintAxis, 20.0f);
		littleF.setFixedBaseMode(true);
		handStructureModel.connectChain(littleF, 9, 0, BoneConnectionPoint.END);
	}

	/**
	 * @param lenghtMultplier
	 * @param boneDirection
	 * @param handBaseBoneEnd
	 * @param baseColor
	 */
	private void setupMiddleFinger(Vec3f handBaseBoneEnd, Colour4f baseColor) {
		// middle Finger------------------------------------------------------------
		Vec3f middelfFConEnd = new Vec3f(0.0f, 5.0f * lenghtMultiplier, 0.0f);
		Vec3f middleFBaseStart = new Vec3f();
		Vec3f middleFBaseEnd = middleFBaseStart.plus(boneDirection.times(5.0f * lenghtMultiplier));
		FabrikBone3D middleFConnect = new FabrikBone3D(handBaseBoneEnd, middelfFConEnd);
		middleFConnect.setColour(baseColor);
		FabrikChain3D middleFConChain = new FabrikChain3D();
		middleFConChain.addBone(middleFConnect);
		handStructureModel.addChain(middleFConChain);

		// middlef MP
		FabrikBone3D middleFBase = new FabrikBone3D(middleFBaseStart, middleFBaseEnd);
		cols[2] = new Colour4f(Utils.MID_BLUE);
		middleFBase.setColour(cols[2].lighten(0.4f));
		middleF.setMaxIterationAttempts(10);
		middleF.addBone(middleFBase);
		// middle PIP
		middleF.addConsecutiveHingedBone(boneDirection, 4.0f * lenghtMultiplier, JointType.LOCAL_HINGE, X_AXIS, 0, 95,
				Y_AXIS, cols[2].darken(0.4f));
		// middle DIP
		middleF.addConsecutiveHingedBone(boneDirection, 2.0f * lenghtMultiplier, JointType.LOCAL_HINGE, X_AXIS, 0, 95,
				Y_AXIS, cols[2].lighten(0.4f));

		fingers[2] = middleF;
		middleF.setRotorBaseboneConstraint(BaseboneConstraintType3D.LOCAL_ROTOR, boneDirection, 15.0f);
		middleF.setFixedBaseMode(true);
		handStructureModel.connectChain(middleF, 5, 0, BoneConnectionPoint.END);

	}

	/**
	 * @param lenghtMultplier
	 * @param boneDirection
	 * @param handBaseBoneEnd
	 * @param baseColor
	 */
	private void setupRingFinger(Vec3f handBaseBoneEnd, Colour4f baseColor) {
		// ring finger------------------------------------------------------------
		Vec3f ringFConnEnd = new Vec3f(2.5f * lenghtMultiplier, 4.0f * lenghtMultiplier, 0.0f);
		Vec3f ringFBaseStart = new Vec3f();
		Vec3f ringFBaseEnd = ringFBaseStart.plus(boneDirection.times(5.0f * lenghtMultiplier));
		FabrikBone3D ringFConnect = new FabrikBone3D(handBaseBoneEnd, ringFConnEnd);
		ringFConnect.setColour(baseColor);
		FabrikChain3D ringFConChain = new FabrikChain3D();
		ringFConChain.addBone(ringFConnect);
		handStructureModel.addChain(ringFConChain);

		// ring MP
		FabrikBone3D ringFBase = new FabrikBone3D(ringFBaseStart, ringFBaseEnd);
		cols[3] = new Colour4f(Utils.CYAN);
		ringFBase.setColour(cols[3].darken(0.8f));
		ringF.setMaxIterationAttempts(10);
		ringF.addBone(ringFBase);
		// ring PIP
		ringF.addConsecutiveHingedBone(boneDirection, 3.0f * lenghtMultiplier, JointType.LOCAL_HINGE, X_AXIS, 0, 95,
				Y_AXIS, cols[3].lighten(0.4f));
		// ring DIP
		ringF.addConsecutiveHingedBone(boneDirection, 2.0f * lenghtMultiplier, JointType.LOCAL_HINGE, X_AXIS, 0, 95,
				Y_AXIS, cols[3].darken(0.4f));
		fingers[3] = ringF;
		ringF.setRotorBaseboneConstraint(BaseboneConstraintType3D.LOCAL_ROTOR, boneDirection, 15.0f);
		ringF.setFixedBaseMode(true);
		handStructureModel.connectChain(ringF, 7, 0, BoneConnectionPoint.END);
	}

	public void updateTrackedObjectPosition() {
		// //TODO get object position
		Mat4f mat = htcVivetracker.lastTrackingResult;
		// reordered Axis components to match optical axis system
		Vec3f tempVec = new Vec3f(mat.m03 - objectCalibrationPos.x, mat.m13 - objectCalibrationPos.y,
				mat.m23 - objectCalibrationPos.z);
		Vec3f trans = new Vec3f(tempVec.z * 100.0f, tempVec.x * 100.0f, tempVec.y * 100.0f);
		trackingObjBone.setStartLocation(trans);
		Vec3f.add(trans, new Vec3f(0.0f, 0.0f, 1.0f));
		trackingObjBone.setEndLocation(trans);
	}
	/*
	*function updates hand model positions before ik calculation
	*/
	public void updateStructurePos(FabrikStructure3D structure, Vec3f handBasePos) {
		Vec3f delta = handBasePos;
		handData.setHandBasePos(new Vector3f(handBasePos.x, handBasePos.y, handBasePos.z));
		Vec3f test = handBaseChain.getBaseLocation();
		Vector3f temp = handData.getHandBasePos();
		temp.sub(new Vector3f(test.x, test.y, test.z));
		Vec3f.subtract(delta, test);

		int numChains = structure.getNumChains();
		for (int i = 0; i < numChains; i++) {
			FabrikChain3D chain = structure.getChain(i);
			for (int k = 0; k < chain.getNumBones(); k++) {

				try {
					FabrikBone3D bone = chain.getBone(k);
					Vec3f sl = bone.getStartLocation();
					Vec3f el = bone.getEndLocation();
					Vec3f.add(sl, delta);
					Vec3f.add(el, delta);
					bone.setStartLocation(sl);
					bone.setEndLocation(el);
				} catch (Exception e) {
					continue;
				}
			}
		}
	}

	public void update() {

		KeyboardHandler.isKeyDown(GLFW_KEY_ESCAPE);
		if (KeyboardHandler.isKeyDown(GLFW_KEY_ESCAPE)) {
			glfwSetWindowShouldClose(window, true);
			calc.udpthreadLeft.interrupt();
			calc.udpthreadRight.interrupt();
			System.exit(0);
		}

		try {
			// Retrieve current Data
			Vec2f[] rVecF = calc.getDatasetFiltered(0);
			Vec2f[] lVecF = calc.getDatasetFiltered(1);
			long er = calc.getEpochRight();
			long el = calc.getEpochLeft();
			long ed = el - er;
			if (Math.abs(ed) < 20.0) {
				Vec2f[] rVec = calc.getDatasetUnfiltered(0);
				Vec2f[] lVec = calc.getDatasetUnfiltered(1);
				// only draw frames with max difference of 1 frame

				// Calculate Z distance
				for (int i = 0; i < rVec.length; i++) {

					float dist = calculateDepthData(rVec[i], lVec[i]);
					handData.updateCurrentFingerPos(i, setPositionData(rVecF[i], lVecF[i], dist));
				}
				try {
					updateStructurePos(handStructureModel, calculateRealDistance(handData.getCurrentFingerPos()[5]));
				} catch (Exception e2) {
				}
			}
		} catch (Exception e) {
			e.printStackTrace();

			 System.out.println("No tracking data available. Please check network connections and System status before restart.");

		}
		//Keyboard input handling for camra control
		if (KeyboardHandler.isKeyDown(GLFW_KEY_Q)) {

			Mat4f rotMat = new Mat4f();
			rotMat.setIdentity();
			rotMat = rotMat.rotateAboutLocalAxisDegs(0.1f, Y_AXIS);
			projectionViewMatrix = projectionViewMatrix.times(rotMat);
		}
		if (KeyboardHandler.isKeyDown(GLFW_KEY_E)) {
			Mat4f rotMat = new Mat4f();
			rotMat.setIdentity();
			rotMat = rotMat.rotateAboutLocalAxisDegs(-0.1f, Y_AXIS);
			projectionViewMatrix = projectionViewMatrix.times(rotMat);
		}
		if (KeyboardHandler.isKeyDown(GLFW_KEY_W)) {

			viewMatrix.setIdentity();
			viewMatrix = viewMatrix.translate(0.0f, 0.0f, 0.10f);
			projectionViewMatrix = projectionViewMatrix.times(viewMatrix);
		}
		if (KeyboardHandler.isKeyDown(GLFW_KEY_S)) {

			viewMatrix.setIdentity();
			viewMatrix = viewMatrix.translate(0.0f, 0.0f, -0.10f);
			projectionViewMatrix = projectionViewMatrix.times(viewMatrix);
		}
		if (KeyboardHandler.isKeyDown(GLFW_KEY_A)) {
			viewMatrix.setIdentity();
			viewMatrix = viewMatrix.translate(0.1f, 0.0f, 0.0f);
			projectionViewMatrix = projectionViewMatrix.times(viewMatrix);
		}
		if (KeyboardHandler.isKeyDown(GLFW_KEY_D)) {

			viewMatrix.setIdentity();
			viewMatrix = viewMatrix.translate(-0.10f, 0.0f, 0.0f);
			projectionViewMatrix = projectionViewMatrix.times(viewMatrix);
		}
		if (KeyboardHandler.isKeyDown(GLFW_KEY_R)) {

			viewMatrix.setIdentity();
			viewMatrix = viewMatrix.translate(0.0f, -0.10f, 0.0f);
			projectionViewMatrix = projectionViewMatrix.times(viewMatrix);
		}
		if (KeyboardHandler.isKeyDown(GLFW_KEY_F)) {

			viewMatrix.setIdentity();
			viewMatrix = viewMatrix.translate(0.0f, 0.10f, 0.0f);
			projectionViewMatrix = projectionViewMatrix.times(viewMatrix);
		}


		//After tracker is placed in the tracking space, a push to c offsets it's current position as origin
		if (KeyboardHandler.isKeyDown(GLFW_KEY_C)) {

			if (!trackerCalibrated) {
				Mat4f mat = htcVivetracker.lastTrackingResult;
				objectCalibrationPos = new Vec3f(mat.m03, mat.m13, mat.m23);
				trackerCalibrated = true;
			}

		}
		if (KeyboardHandler.isKeyDown(GLFW_KEY_H)) {

			System.out.println(htcVivetracker.lastTrackingResult);
		}
		//Update HTC tracker data at the end of the update loop
		try {
			SimpleOpenVRWrapper.update();
		} catch (Exception e) {
			System.err.println("OpenVr wrapper update failed");
			e.printStackTrace();
		}
		try {
			htcVivetracker.update();
		} catch (Exception e) {
			System.err.println("Vive Tracker update failed");
			e.printStackTrace();
		}
		updateTrackedObjectPosition();

	}

	/**
	 * @param rightSideDataUnfiltered
	 * @param leftSideData
	 * @param targetNumber
	 */
	private float calculateDepthData(Vec2f rightSideDataUnfiltered, Vec2f leftSideDataUnfiltered) {
		return calc.calculateZDistance(rightSideDataUnfiltered.x, leftSideDataUnfiltered.x);
	}

	public Vec3f setPositionData(Vec2f rightSideDataFiltered, Vec2f leftSideDataFiltered, float zDist) {
		float targetX =  rightSideDataFiltered.x;
		float targetY =  rightSideDataFiltered.y;
		return new Vec3f(targetX, targetY, zDist);

	}

	public Vec3f calculateRealDistance(Vec3f pixelPos) {
		double height = pixelPos.z;
		double correctionFactor = (90.0f - height) / 90.0f;
		double correctedX = pixelConversionFactorX * correctionFactor * pixelPos.x;
		double correctedY = pixelConversionFactorY * correctionFactor * pixelPos.y;
		return new Vec3f((float) correctedX, (float) correctedY, pixelPos.z);
	}

	private void loop() throws SocketException {

		// Set the clear color
		glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
		FabrikConstraint3D constraint = new FabrikConstraint3D();
		// Run the rendering loop until the user has attempted to lose
		// the window or has pressed the ESCAPE key.

		while (!glfwWindowShouldClose(window)) {
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear buffers
			// Draw reference coordinate system
			axis.draw(projectionViewMatrix);
			// Draw bone Coordinate systems -> DEBUG
			// structureAxis.draw(handStructureModel, projectionViewMatrix, viewMatrix);
			for (int i = 0; i < fingers.length; i++) {
				try {
					drawTargetAndSolve(i);
				} catch (Exception e) {
					continue;
				}
			}
			try {
				//Draw hand structure
				model.drawStructure(handStructureModel, projectionViewMatrix, viewMatrix, Utils.BLACK);
				//draw the model for the tracker at the supplied position
				try {
					trackingObjModel.drawBone(trackingObjBone, projectionViewMatrix, viewMatrix, Utils.RED);
				} catch (Exception e)
				{
					e.printStackTrace();
				}
				//DEBUG draw of bone structure as only colored lines
				// FabrikLine3D.draw(handStructureModel, 1.0f, projectionViewMatrix);
			} catch (Exception e1) {
				// TODO Auto-generated catch block
				System.out.println("hand model draw error");
				e1.printStackTrace();
				System.exit(0);
			}
			// DEBUG draw of applied movement contraint visuals
			// constraint.draw(handStructureModel, 2.0f, projectionViewMatrix);
			glfwSwapBuffers(window); // Swap color buff.

			// Poll for window events. The key callback above will only be // invoked during
			// this call.
			glfwPollEvents();
			update();
		}

	}

	/**
	 * @param i
	 */
	private void drawTargetAndSolve(int i) {
		try {
			Point3D targetPoint = new Point3D();
			Vec3f currPos = calculateRealDistance(handData.getCurrentFingerPos()[i]);
			try {
				targetPoint.draw(currPos, Utils.GREEN, 10.0f, projectionViewMatrix);
			} catch (Exception e) {
			}

			FabrikChain3D chainToSolveFor = handStructureModel.getChain(2 + (i * 2));
			chainToSolveFor.solveForTarget(currPos);
		} catch (Exception e) {
			// System.out.println("not solved");
			// e.printStackTrace();
		}
	}

	public static void main(String[] args) throws SocketException, InterruptedException {
		new IkSolver().run();
	}

}
