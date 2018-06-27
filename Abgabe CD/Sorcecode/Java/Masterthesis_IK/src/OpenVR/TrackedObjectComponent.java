package OpenVR;

import OpenVR.HelperClass;
import au.edu.federation.utils.Mat4f;
import au.edu.federation.utils.Vec3f;

import org.lwjgl.openvr.HmdMatrix34;
import org.lwjgl.openvr.HmdMatrix44;
import org.lwjgl.openvr.TrackedDevicePose;
import org.lwjgl.openvr.VRSystem;

import static org.lwjgl.openvr.VR.ETrackingUniverseOrigin_TrackingUniverseStanding;
import static org.lwjgl.openvr.VR.k_unMaxTrackedDeviceCount;

//This Component transfers the transform of a specified TrackedDevice and applies it to the Transform of its parent SceneObject,
//optionally corrected by a correction matrix.
public class TrackedObjectComponent {

    public Mat4f  correctionMatrixTranslation;
    public Mat4f  correctionMatrixRotation;
    public Mat4f  lastTrackingResult;
    private int  trackedDeviceType;


    //deviceType: for Example (ETrackedDeviceClass_TrackedDeviceClass_GenericTracker)
    public TrackedObjectComponent( int deviceType)
    {
        correctionMatrixTranslation = new Mat4f();
        correctionMatrixRotation    = new Mat4f();
        trackedDeviceType = deviceType;
        lastTrackingResult = new Mat4f();
        correctionMatrixRotation.setIdentity();
        correctionMatrixTranslation.setIdentity();
    }

    public void update()
    {
        TrackedDevicePose.Buffer trackedPosesBuffer = TrackedDevicePose.calloc(k_unMaxTrackedDeviceCount);

        try {
			VRSystem.VRSystem_GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin_TrackingUniverseStanding, 0f, trackedPosesBuffer);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        //TODO: This is definitely not the fastest way to do it. Could lead to mixups with multiple devices of the same type
        for(int i = 0; i < trackedPosesBuffer.sizeof(); i++) {
            int currentDeviceType = VRSystem.VRSystem_GetTrackedDeviceClass(i);

            if(currentDeviceType == trackedDeviceType)
            {
                //Compositor not working
                TrackedDevicePose currentPose = trackedPosesBuffer.get(i);
                if(currentPose.bPoseIsValid())
                {
                    HmdMatrix34 vrMatOVR = currentPose.mDeviceToAbsoluteTracking();
                    Mat4f vrMat = new Mat4f();
                    HelperClass.transferOpenVRMatrix(vrMat, vrMatOVR);
                    //lastTrackingResult = vrMat;
                    Mat4f newRotation  = vrMat.times( correctionMatrixRotation);
                 // resulting matrix to use
                    Mat4f newTransform = newRotation.times(correctionMatrixTranslation);
                    lastTrackingResult=newTransform;
                    
                }
            }
        }


        trackedPosesBuffer.free();
    }
}
