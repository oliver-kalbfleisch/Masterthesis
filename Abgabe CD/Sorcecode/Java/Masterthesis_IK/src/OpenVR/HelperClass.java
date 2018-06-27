package OpenVR;

import org.lwjgl.openvr.HmdMatrix34;
import org.lwjgl.openvr.HmdMatrix44;

import au.edu.federation.utils.Mat4f;

import java.nio.FloatBuffer;

public class HelperClass {


    public static void transferOpenVRMatrix(Mat4f output, HmdMatrix44 input)
    {
        FloatBuffer fb = input.m();
        output.m00 = fb.get(0);
        output.m01 = fb.get(1);
        output.m02 = fb.get(2);
        output.m03 = fb.get(3);

        output.m10 = fb.get(4);
        output.m11 = fb.get(5);
        output.m12 = fb.get(6);
        output.m13 = fb.get(7);

        output.m20 = fb.get(8);
        output.m21 = fb.get(9);
        output.m22 = fb.get(10);
        output.m23 = fb.get(11);

        output.m30 = fb.get(12);
        output.m31 = fb.get(13);
        output.m32 = fb.get(14);
        output.m33 = fb.get(15);
    }

    public static void transferOpenVRMatrix(Mat4f output, HmdMatrix34 input)
    {
        FloatBuffer fb = input.m();
        output.m00 = fb.get(0);
        output.m01 = fb.get(1);
        output.m02 = fb.get(2);
        output.m03 = fb.get(3);

        output.m10 = fb.get(4);
        output.m11 = fb.get(5);
        output.m12 = fb.get(6);
        output.m13 = fb.get(7);

        output.m20 = fb.get(8);
        output.m21 = fb.get(9);
        output.m22 = fb.get(10);
        output.m23 = fb.get(11);

        output.m30 = 0;
        output.m31 = 0;
        output.m32 = 0;
        output.m33 = 1;
    }


}
