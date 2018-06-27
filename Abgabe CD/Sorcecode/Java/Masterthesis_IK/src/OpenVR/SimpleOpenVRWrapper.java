package OpenVR;
import static org.lwjgl.openvr.VR.ETrackingUniverseOrigin_TrackingUniverseStanding;
import static org.lwjgl.openvr.VR.VR_GetVRInitErrorAsEnglishDescription;
import static org.lwjgl.openvr.VR.VR_InitInternal;
import static org.lwjgl.openvr.VR.VR_IsRuntimeInstalled;
import static org.lwjgl.openvr.VR.VR_ShutdownInternal;
import static org.lwjgl.openvr.VR.k_unMaxTrackedDeviceCount;
import static org.lwjgl.system.MemoryUtil.memAllocInt;
import static org.lwjgl.system.MemoryUtil.memFree;

import java.nio.IntBuffer;

import org.lwjgl.openvr.OpenVR;
import org.lwjgl.openvr.TrackedDevicePose;
import org.lwjgl.openvr.VR;
import org.lwjgl.openvr.VRSystem;


//Due to the way VRCompositor_WaitGetPoses() works, this component should be updated at the very end, but prior to rendering.
public class SimpleOpenVRWrapper{


    private static   boolean openVRInitialized = false;



     

    public static boolean initOpenVR()
    {

        if(!openVRInitialized) {
            boolean hmdPresent = VR.VR_IsHmdPresent();
            boolean runtimePresent = VR_IsRuntimeInstalled();
            System.out.println("Initializing OpenVR...");
            System.out.println("Runtime installed: " + runtimePresent);
            System.out.println("HMD detected: " + hmdPresent);

            IntBuffer error = memAllocInt(1);
            int token = 0;
            token = VR_InitInternal(error, VR.EVRApplicationType_VRApplication_Other);

            String errorString = VR_GetVRInitErrorAsEnglishDescription(error.get(0));
            memFree(error);

            if (token == 0) {
                System.out.println(errorString +" token");
                
                return false;
            } else {
                OpenVR.create(token);
                openVRInitialized = true;
                return true;
            }
        } else {
            return true;
        }
    }

    
    public static void update()
    {
        TrackedDevicePose.Buffer trackedPosesBuffer       = TrackedDevicePose.calloc(k_unMaxTrackedDeviceCount);
        VRSystem.VRSystem_GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin_TrackingUniverseStanding, 0f, trackedPosesBuffer);
        trackedPosesBuffer.free();
    }


    public static void end()
    {
        VR_ShutdownInternal();
    }

}
