#include "main.h"
using namespace cv;
using namespace std;

namespace pt= boost::property_tree;
const int numThreads=4;

const int frame_width=640;
const int frame_height=480;
const unsigned int x_offset= 40;
const unsigned int y_offset= 40;
const int cameraSide=1;

Mat *imgOriginal (new Mat());
//CHANGE HOST IP TO MATCH SETUP IF NEEDED
const string host="192.168.1.10";
// Set correct value for camera -Left:8888 -Right :9999
const string port= "9999";
const string multicast_port="6666";
//Clock for time measurement
clock_t t;
//Stringstreams for outputting data
stringstream ss;
//stringstream logger;
//ofstream outfile("single_thread_dynamic.txt");

// initital color calibration gui values
int numberOfTrackedColors= 6;
int low_r=30;
int low_g=30;
int low_b=30;

int high_r = 100;
int high_g = 100;
int high_b = 100;

int saturation=100;
int brightness=43;
int gain=27;
int exposure=2;
int contrast=47;
int awb_red=0;
int awb_blue=0;
string windowTitle;

//STEREO CALIBRATION VARIABLES
//Extrinsics
Mat R, R1, R2, P1, P2, Q;
//Intrinsics
Mat M1, M2, D1, D2;
//Remap
Mat rmap[2][2];

//Main Components
vector<Point2i> *contourCenters=new vector<Point2i>(numberOfTrackedColors);
vector<Rect> *imageROIS =new vector<Rect>(numberOfTrackedColors);
vector<Point2i>*offsets= new vector<Point2i>(numberOfTrackedColors);
float angle=0.0;
vector<vector<int> > *colorThreshold=new vector<vector<int> >(numberOfTrackedColors);

raspicam::RaspiCam_Cv *Camera = new raspicam::RaspiCam_Cv;
//Multithreading

//setup threadpool
//boost::thread_group threadpool;
//vector< boost::future<void> > pending_data;

//--------------------------------------------------------------------------------------------------------------

//function blurrs and erodes/dilliates the input image to remove high frequncy nose in the image
void image_optimizations(Mat *imgThresholded)
{
    try {
        GaussianBlur(*imgThresholded,*imgThresholded,Size(3,3),0,0);
    } catch(...) {
        cout<<(*imgThresholded).size()<<endl;
        cout<<"error applying gaussian blurr\n";
    }
    try {
        erode(*imgThresholded, *imgThresholded, getStructuringElement(MORPH_RECT, Size(4, 4)) );
        dilate(*imgThresholded, *imgThresholded, getStructuringElement(MORPH_RECT, Size(4, 4)) );
    } catch(...) {
        cout<<"error in morphological opening\n";
    }
    try {
        dilate( *imgThresholded, *imgThresholded, getStructuringElement(MORPH_RECT, Size(4, 4)) );
        erode(*imgThresholded, *imgThresholded, getStructuringElement(MORPH_RECT, Size(4, 4)) );
    } catch(...) {
        cout<<"error in morphological closing\n";
    }

}
//function takes in HSV image and color boundaries for detection
void detect_color(Mat image,vector<int> *thresholds,Mat *imgThresholded)
{
    //FOR DEGUB USE ONLY, REMOVE FOR PRODUCTIVE
    //imshow("threshold image",image);
    //--------------------------------------

    try {
        inRange(image, Scalar((*thresholds)[4],(*thresholds)[2], (*thresholds)[0]), Scalar((*thresholds)[5], (*thresholds)[3], (*thresholds)[1]), *imgThresholded);
    } //Threshold the image
    catch(...)
    {
        cout<<"exception in inRange"<<endl;
    }


}

// function returns center point of color contour drived from input mask
int detectContour(Mat *mask,Point2i *center, Rect *roi,Point2i *offset,float *angle)
{
    vector<vector<Point> > contourPoints;
    //find contour points in mask
    try {
        findContours(*mask,contourPoints,CV_RETR_LIST, CV_CHAIN_APPROX_NONE );
    }
    catch(...)
    {   
        cout<<"could not find contours\n";
        *center= *center+*offset;
        *offset=Point2i(0,0);
        *roi=Rect(0,0,frame_width,frame_height);
        return -1;
    }
    unsigned int largest_area=0;
    int largest_contour_index=-1;

    for( unsigned int i = 0; i< contourPoints.size(); i++ )
    {   //Find the area of contour
        float area = contourArea( contourPoints[i] );

        if( area > largest_area )
        {
            largest_area = area;
            //Store the index of largest contour
            largest_contour_index = i;
        }
    }
    if(largest_contour_index >= 0)
    {
        //calculate minimum rotated bounding rect
        RotatedRect minRect = minAreaRect(Mat(contourPoints[largest_contour_index]));
        *angle=minRect.angle;
        Rect bRect=minRect.boundingRect();

        /* ROI AREA CALULATION
         # tl------tl+w
         #  +       +
         #  +       +
         #  +       +
         #  tl+h-----br
         */
        unsigned int x0=boost::algorithm::clamp((offset->x+bRect.tl().x-x_offset),0,frame_width);
        unsigned int y0=boost::algorithm::clamp((offset->y+bRect.tl().y-y_offset),0,frame_height);

        unsigned int corr=0;
        unsigned int pos_right=x0+bRect.size().width+(2*(x_offset));
        if(pos_right > frame_width)
        {
            corr=pos_right-frame_width;
        }
        unsigned int width_clamped=boost::algorithm::clamp(bRect.size().width+(2*(x_offset))-corr,0,frame_width);
        unsigned int pos_bottom=y0+bRect.size().height+(2*(y_offset));
        if(pos_bottom > frame_height)
        {
            corr=pos_bottom-frame_height;
        }
        unsigned int height_clamped=boost::algorithm::clamp(bRect.size().height+(2*(y_offset))-corr,0,frame_height);
        *roi=Rect(x0,y0,width_clamped,height_clamped);
        *offset=Point2i(x0,y0);
        *center= minRect.center;

    }
    else
    {
        *angle=-90.0;
       // *center= *center+*offset;
       // *offset=Point2i(0,0);
        *roi=Rect(0,0,frame_width,frame_height);
        return -1;
    }
    return 0;

}
//Function to retrieve Camera frame from circular buffer
void getCameraFrame(circular_buffer<Mat> *buff,raspicam::RaspiCam_Cv *Camera )
{
    Mat imgOriginal;
    while(true) {
        bool grabbed=Camera->grab();
        if(!grabbed)
        {
            cout<<"could not grab frame from camera"<<endl;
        }
        Camera->retrieve(imgOriginal);
        buff->put(imgOriginal);
    }
}
//Function can be used for single and Multithreaded approach. It contains all needed parts to detect a color amrker in the image
/**
*@param colorBoundary -> Pointer to Vector containing the RGB upper and lower threshold values
*@param imgOriginal -> Original or ROI frmae from the camera
*@param contourCenter -> Pointer to Vector element for storing the calculated contour center
*@param roi -> Pointer to Vector element for storing ROI Data
*@param offset -> Pointer to vector element for storing the roi area offset data
*@param angle -> Pointer to vector element for storing the angle of the bounding box
*/ 
void color_detectThread(vector<int> &color_boundary, Mat imgOriginal, Point2i &contourCenter,Rect &roi, Point2i &offset,float &angle)
{
    int count=0;
    Mat imgThresholded;
    try {
        detect_color(imgOriginal,&color_boundary, &imgThresholded);
        count++;
    }
    catch(...)
    {
        cout<<"error in color detect\n";
    }
    //If a contour could be found continue, else skip
    if(imgThresholded.size().width>0 && imgThresholded.size().height>0) {
        try {
            image_optimizations(&imgThresholded);
            count++;
        }
        catch(...)
        {
            cout<<"error in image opt\n";
        }

        try {
            detectContour(&imgThresholded, &contourCenter,&roi,&offset,&angle);
            count++;
        }
        catch(...)
        {
            cout<<"error in detect contour";
        }
    }
    else {
        cout<<"skipped processing because of missing contour detection\n";
        //reseting tracking pos
        *&contourCenter= *&contourCenter+*&offset;
        *&offset=Point2i(0,0);
        *&roi=Rect(0,0,frame_width,frame_height);
    }
}
//Utility function for the calibration GUI
void low_r_thresh(int, void *)
{
    low_r=min(high_r-1,low_r);
    setTrackbarPos("Low R",::windowTitle,low_r);
}
void high_r_thresh(int,void *)
{
    high_r=max(high_r,low_r+1);
    setTrackbarPos("High R",::windowTitle,high_r);
}
void low_g_thresh(int, void *)
{
    low_g=min(high_g-1,low_g);
    setTrackbarPos("Low G",::windowTitle,low_g);
}
void high_g_thresh(int,void *)
{
    high_g=max(high_g,low_g+1);
    setTrackbarPos("High G",::windowTitle,high_g);
}
void low_b_thresh(int, void *)
{
    low_b=min(high_b-1,low_b);
    setTrackbarPos("Low B",::windowTitle,low_b);
}
void high_b_thresh(int,void *)
{
    high_b=max(high_b,low_b+1);
    setTrackbarPos("High B",::windowTitle,high_b);
}
void saturation_thresh(int, void*)
{
    setTrackbarPos("Saturation","Camera Calibration",saturation);
}
void brightness_thresh(int, void*)
{
    setTrackbarPos("Brightness","Camera Calibration",brightness);
}
void gain_thresh(int, void*)
{
    setTrackbarPos("Gain","Camera Calibration",gain);
}
void exposure_thresh(int, void*)
{
    setTrackbarPos("Exposure","Camera Calibration",exposure);
}
void contrast_thresh(int, void*)
{
    setTrackbarPos("Contrast","Camera Calibration",contrast);
}
void awb_red_thresh(int, void*)
{
    setTrackbarPos("awb red","Camera Calibration",awb_red);
}
void awb_blue_thresh(int, void*)
{
    setTrackbarPos("awb blue","Camera Calibration",awb_blue);
}
void callbackButton(int state,void* userdata)
{
    cout<<"button pressed!"<<state<<userdata<<endl;
}
void setupCameraTrackbars()
{
    namedWindow("Camera Calibration",WINDOW_NORMAL);
    createTrackbar("Saturation","Camera Calibration",&saturation,100,saturation_thresh);
    createTrackbar("Brightness","Camera Calibration",&brightness,100,brightness_thresh);
    createTrackbar("Gain","Camera Calibration",&gain,100,gain_thresh);
    createTrackbar("Contrast","Camera Calibration",&contrast,100,contrast_thresh);
    createTrackbar("Exposure","Camera Calibration",&exposure,100,exposure_thresh);
    createTrackbar("awb red","Camera Calibration",&awb_red,100,awb_red_thresh);
    createTrackbar("awb blue","Camera Calibration",&awb_blue,100,awb_blue_thresh);
}
void setupColorTrackbars()
{
    namedWindow(::windowTitle,WINDOW_NORMAL);
    createTrackbar("Low R",::windowTitle,&low_r,255,low_r_thresh);
    createTrackbar("High R",::windowTitle,&high_r,255,high_r_thresh);
    createTrackbar("Low G",::windowTitle,&low_g,255,low_g_thresh);
    createTrackbar("High G",::windowTitle,&high_g,255,high_g_thresh);
    createTrackbar("Low B",::windowTitle,&low_b,255,low_b_thresh);
    createTrackbar("High B",::windowTitle,&high_b,255,high_b_thresh);
}

//utility function for saving the calibration data to JSOn files
void saveCameraDataToJSOn(string fileName,float version,vector<int> *calibData)
{
    pt::ptree dataRoot;
    //Add metadta
    dataRoot.put("version",version);
    dataRoot.put("Saturation",calibData->at(0));
    dataRoot.put("Brightness",calibData->at(1));
    dataRoot.put("Gain",calibData->at(2));
    dataRoot.put("Contrast",calibData->at(3));
    dataRoot.put("Exposure",calibData->at(4));
    dataRoot.put("awb red",calibData->at(5));
    dataRoot.put("awb blue",calibData->at(6));
    ofstream out(fileName);
    pt::write_json(out,dataRoot);
    out.close();
}
void saveColorDataToJSON(string fileName,float version, string colorMode, int numTrackedColors,vector< vector<int> > *&colorThreshold)
{
    pt::ptree dataRoot;
    //Add metadta
    dataRoot.put("version",version);
    dataRoot.put("colorMode",colorMode);
    dataRoot.put("numTrackedColors",numTrackedColors);
    //Add colors array data
    pt::ptree colors_node;

    for(int i=0; i<numTrackedColors; i++)
    {
        //vector structure RL,RH,GL,GH,BL,BH
        vector<int> colorData=(*colorThreshold)[i];
        pt::ptree color;
        color.put("colorID",i);
        color.put("name","changeme");
        color.put("rUpper",colorData[1]);
        color.put("rLower",colorData[0]);
        color.put("gUpper",colorData[3]);
        color.put("gLower",colorData[2]);
        color.put("bUpper",colorData[5]);
        color.put("bLower",colorData[4]);
        colors_node.push_back(make_pair("",color));


    }
    dataRoot.add_child("colors",colors_node);
    ofstream out(fileName);
    pt::write_json(out,dataRoot);
    out.close();
}
//Utility functions for the console print out
void setupCameraCalibration(Mat *imgOriginal,raspicam::RaspiCam_Cv *Camera )
{
    cout<<"*\n";
    cout<<"Do you want to check Camera settings befor starting?\n This feature requires a display to be connected. (y/n) ";
    char resp;
    cin>>resp;
    switch(resp)
    {
    case 'y':
    {
        cout<<"*\n";
        cout<<"System will now switch to camera calibration mode. Press enter to set values when finished.";
        cout<<"*\n";
        setupCameraTrackbars();
        //Setup Camera
        if ( !Camera->open())
        {
            cout<<"*"<<endl;
            cout << "Cannot open the web cam" << endl;
            return;
        }
        cout<<"*"<<endl;
        cout<<"*Camera warmup phase started"<<endl;
        sleep(5);
        cout<<"*"<<endl;
        cout<<"*Warmup finished"<<endl;
        bool calibrating=true;
        while (calibrating) {
            //grab camera Frame
            bool grabbed=Camera->grab();
            if(grabbed)
            {
                try {
                    Camera->retrieve(*imgOriginal);
                } catch(...)
                {
                    cout<<"*"<<endl;
                    cout<<"*Camera retrieve error"<<endl;
                }
            }
            Camera->set(CV_CAP_PROP_SATURATION,saturation);
            Camera->set(CV_CAP_PROP_GAIN,gain);
            Camera->set(CV_CAP_PROP_BRIGHTNESS,brightness);
            Camera->set(CV_CAP_PROP_EXPOSURE,exposure);
            Camera->set(CV_CAP_PROP_CONTRAST,contrast);
            Camera->set(CV_CAP_PROP_WHITE_BALANCE_RED_V,awb_red);
            Camera->set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U,awb_blue);
            imshow("Camera Calibration",*imgOriginal);
            if((char)10==(char)waitKey(1)) {
                destroyAllWindows();
                Camera->release();
                calibrating=false;
                cout<<"*Calibration finished, using new values."<<endl;
                cout<<"*\n";
                cout<<"Do you want to save the new Values to a JSON file? (y/n)";
                char saveFile;
                cin>>saveFile;
                switch(saveFile)
                {
                case 'n':
                {
                    cout<<"*\n"<<"System will use new values without saving to file.\n";
                    break;
                }
                case 'y':
                {
                    cout<<"*\n"<<"Please enter filename for new file in format <filename>.json : ";
                    string userFileName;
                    cin>>userFileName;
                    cout<<"New JSON file "<<userFileName<<" will be saved to directory of the executable.\n";
                    vector<int> cameraCalibData;
                    cameraCalibData.push_back (saturation);
                    cameraCalibData.push_back (brightness);
                    cameraCalibData.push_back (gain);
                    cameraCalibData.push_back (contrast);
                    cameraCalibData.push_back (exposure);
                    cameraCalibData.push_back (awb_red);
                    cameraCalibData.push_back (awb_blue);
                    saveCameraDataToJSOn(userFileName,1.0,&cameraCalibData);
                    break;
                }
                }
            }
        }
    }
    case 'n':
    {
        cout<<"*\n";
        cout<<"*System will use predefined values for camera calibration.\n ";
        break;
    }
    }
}
void setupColorCalibration(vector<vector<int> > *colorThreshold,Mat *imgOriginal,raspicam::RaspiCam_Cv *Camera )
{
    cout<<"*"<<endl;
    cout<<"*System will now switch to color calibration mode."<<endl;
    cout<<"*\n";
    cout<<"How many colors do you want to detect (max. 6)? ";
    char resp;
    cin>>resp;
    int numInput=resp-'0';
    if(numInput>0)
    {
        //Reassign pointer value without memory leak
        vector<vector<int> > *temp= new vector<vector<int> >(numInput);
        numberOfTrackedColors=numInput;
        delete colorThreshold;
        colorThreshold=temp;
    }
    else
    {
        cout<<"Invalid input"<<endl;
        return;
    }
    //Setup Camera
    if ( !Camera->open())
    {
        cout<<"*"<<endl;
        cout << "Cannot open the web cam" << endl;
        return;
    }
    cout<<"*"<<endl;
    cout<<"*Camera warmup phase started"<<endl;
    sleep(5);
    cout<<"*"<<endl;
    cout<<"*Warmup finished"<<endl;

    cout<<"*The system will now be displaying a trackbar window with the color \n boundary values and the thresholded image results.Set threshold values to fitting values and finish calibaration for the color by pressing enter.\nThe calibration window for the next color will appear automatically."<<endl;
    Mat res;
    bool calibrating=true;
    int colorCounter=1;
    ::windowTitle="Calibration for Color Nr. "+to_string(colorCounter);
    setupColorTrackbars();
    while(calibrating)
    {
        //grab camera Frame
        bool grabbed=Camera->grab();
        if(grabbed)
        {
            try {
                Camera->retrieve(*imgOriginal);
            } catch(...)
            {
                cout<<"*"<<endl;
                cout<<"*Camera retrieve error"<<endl;
            }
            inRange(*imgOriginal,Scalar(low_b,low_g,low_r),Scalar(high_b,high_g,high_r),res);
            imshow(::windowTitle,res);
        }
        if((char)10==(char)waitKey(1)) {
            //Write calibration values into vector
            (*colorThreshold)[colorCounter-1]= {low_r,high_r,low_g,high_g,low_b,high_b};
            destroyAllWindows();
            colorCounter++;
            ::windowTitle="Calibration for Color Nr. "+to_string(colorCounter);
            setupColorTrackbars();
            if(colorCounter>numInput) {
                Camera->release();
                calibrating=false;
                cout<<"*Calibration finished, using new values."<<endl;
                cout<<"*\n";
                cout<<"Do you want to save the new Values to a JSON file? (y/n)";
                char saveFile;
                cin>>saveFile;
                switch(saveFile)
                {
                case 'n':
                {
                    cout<<"*\n"<<"System will use new values without saving to file.\n";
                    break;
                }
                case 'y':
                {
                    cout<<"*\n"<<"Please enter filename for new file in format <filename>.json : ";
                    string userFileName;
                    cin>>userFileName;
                    cout<<"New JSON file "<<userFileName<<" will be saved to directory of the executable.\n";
                    saveColorDataToJSON(userFileName,1.0,"rgb",numInput,colorThreshold);
                    break;
                }
                }
            }
        }

    }
}
//Utility functions to read data from json files
void readCameraJSONAndSetValues(raspicam::RaspiCam_Cv *Camera,string filepath)
{
    cout<<"*"<<endl;
    cout<<"*Reading JSON Data from cameraCalibration.json to get values ..."<<endl;
    //json file read preps
    pt::ptree jsonRoot;
    pt::read_json(filepath,jsonRoot);
    float version= jsonRoot.get<float>("version");
    cout<<"*"<<endl;
    cout<<"*JSON File version: "<<version<<endl;
    cout<<"*"<<endl;
    saturation=jsonRoot.get<int>("Saturation");
    cout<<"*Saturation: "<<saturation<<endl;
    Camera->set(CV_CAP_PROP_SATURATION,saturation);
    cout<<"*"<<endl;
    brightness=jsonRoot.get<int>("Brightness");
    Camera->set(CV_CAP_PROP_BRIGHTNESS,brightness);
    cout<<"*Brightness: "<<brightness<<endl;
    cout<<"*"<<endl;
    gain=jsonRoot.get<int>("Gain");
    Camera->set(CV_CAP_PROP_GAIN,gain);
    cout<<"*Gain :"<<gain<<endl;
    cout<<"*"<<endl;
    contrast=jsonRoot.get<int>("Contrast");
    Camera->set(CV_CAP_PROP_CONTRAST,contrast);
    cout<<"*Contrast: "<<contrast<<endl;
    exposure=jsonRoot.get<int>("Exposure");
    Camera->set(CV_CAP_PROP_EXPOSURE,exposure);
    cout<<"*Exposure: "<<exposure<<endl;
    cout<<"*"<<endl;
    awb_red=jsonRoot.get<int>("awb red");
    Camera->set(CV_CAP_PROP_WHITE_BALANCE_RED_V,awb_red);
    cout<<"*awb red: "<<awb_red<<endl;
    cout<<"*"<<endl;
    awb_blue=jsonRoot.get<int>("awb blue");
    Camera->set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U,awb_blue);
    cout<<"awb blue: "<<awb_blue<<endl;
    cout<<"*"<<endl;

}
void readColorJSONAndSetValues(vector<vector<int> > *&colorThreshold, string filepath)
{

    cout<<"*"<<endl;
    cout<<"*Reading JSON Data from colorCalibration.json to get values ..."<<endl;
    //json file read preps
    pt::ptree jsonRoot;
    pt::read_json(filepath,jsonRoot);
    float version= jsonRoot.get<float>("version",0);
    cout<<"*"<<endl;
    cout<<"*JSON File version: "<<version<<endl;
    cout<<"*"<<endl;
    string colorMode=jsonRoot.get<string>("colorMode");
    cout<<"*Color Mode :"<<colorMode<<endl;
    cout<<"*"<<endl;
    int numColors=jsonRoot.get<int>("numTrackedColors");
    numberOfTrackedColors=numColors;
    cout<<"*Number of colors tracked: "<<numColors<<endl;
    cout<<"*"<<endl;
    cout<<"*Color boundary values: "<<endl;
    cout<<"*"<<endl;
    //Init Vector for correct number of colors
    vector<vector<int> > *temp= new vector<vector<int> >(numColors);
    delete colorThreshold;
    colorThreshold=temp;
    cout<<"json read "<<colorThreshold->size()<<endl;
    //iterate over Dataset and display values that will be set
    for(pt::ptree::value_type &color : jsonRoot.get_child("colors"))
    {
        cout<<"*Color name: "<<color.second.get<string>("name")<<endl;
        cout<<"*\n";
        int rUpper=color.second.get<int>("rUpper");
        cout<<"*rUpper:"<<rUpper<<endl;
        int rLower=color.second.get<int>("rLower");
        cout<<"*rLower:"<<rLower<<endl;
        cout<<"*\n";
        int gUpper=color.second.get<int>("gUpper");
        cout<<"*gUpper:"<<gUpper<<endl;
        int gLower=color.second.get<int>("gLower");
        cout<<"*gLower:"<<gLower<<endl;
        cout<<"*\n";
        int bUpper=color.second.get<int>("bUpper");
        cout<<"*bUpper:"<<bUpper<<endl;
        int bLower=color.second.get<int>("bLower");
        cout<<"*bLower:"<<bLower<<endl;
        cout<<"*\n*\n";
        // Initialize default color Threshold values
        //vector structure RL,RH,GL,GH,BL,BH
        (*colorThreshold)[color.second.get<int>("colorID")]= {rLower,rUpper,gLower,gUpper,bLower,bUpper};
    }
}

int readAndSetStereoCalibValues(string intrinsic, string extrinsic, Size imageSize)
{
    //READ VALUES FROM GENERATED YAMLS
    FileStorage fs;
    try {
        fs.open(extrinsic, FileStorage::READ);
    }
    catch(...)
    {
        cout<<"Error n firle read."<<endl;
        return -1;
    }

    fs["R"] >> R;
    fs["R1"] >> R1; //left
    fs["R2"] >> R2; //right
    fs["P1"] >> P1; //left
    fs["P2"] >> P2; //right
    fs["Q"]  >> Q;
    fs.release();
    fs.open(intrinsic,FileStorage::READ);
    fs["M1"] >> M1; //left
    fs["M2"] >> M2; //right
    fs["D1"] >> D1; //left
    fs["D2"] >> D2; //right
    fs.release();

    //Precompute map for cv::remap()
    if(cameraSide == 0)
    {
        initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    }
    else if (cameraSide == 1)
    {
        initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
    }
    else
    {
        cout<<"camera side input value incorrect. Aborting";
        return -1;
    }

    return 0;
}
// function to apply a tereo rectification remapping to the supplied image , based on the values from the camera calibration data
int applySteroCalib(Mat *img)
{   //APPLY IMAGE TRANSFORMATION
    Mat rimg;
    try {
        remap(*img, rimg, rmap[cameraSide][0], rmap[cameraSide][1], INTER_LINEAR);
        *img=rimg;
    }

    catch(...)
    {
        std::cout << "error in remapping" << '\n';
        return -1;
    }

    return 0;
}
//Commandline utility functions
int startPrompt(char response)
{
    switch (response) {
    case 'y': {
        readColorJSONAndSetValues(colorThreshold,"colorCalibration.json");
        cout<<numberOfTrackedColors<<endl;
        try {
            vector<Point2i> *cctemp =new vector<Point2i>(numberOfTrackedColors);
            delete contourCenters;
            contourCenters= cctemp;
            vector<Rect> *iroitemp=new vector<Rect>(numberOfTrackedColors);
            delete imageROIS;
            imageROIS=iroitemp;
            vector<Point2i>*offstemp= new vector<Point2i>(numberOfTrackedColors);
            delete offsets;
            offsets=offstemp;
            for(int i=0; i<numberOfTrackedColors; i++)
            {
                imageROIS->at(i)= Rect(0,0,640,480);
                offsets->at(i)= Point2i(0,0);
            }
        } catch(...) {
            cout<<"error in reset"<<endl;
        }
        break;
    }
    case 'n': {
        cout<<"*"<<endl;
        cout<<"*Caution: The color calibration mode cannot be run in headless mode\n and requires a monitor to be attached to the Raspberry"<<endl;
        cout<<"*"<<endl;
        cout<<"*Do you want to continue? (y/n) ";
        char resp;
        cin>>resp;
        switch(resp)
        {
        case 'y': {
            setupCameraCalibration(imgOriginal,Camera);
            setupColorCalibration(colorThreshold,imgOriginal,Camera);
            vector<Point2i> *cctemp =new vector<Point2i>(numberOfTrackedColors);
            delete contourCenters;
            contourCenters= cctemp;
            cout<<contourCenters->size()<<endl;
            vector<Rect> *iroitemp=new vector<Rect>(numberOfTrackedColors);
            delete imageROIS;
            imageROIS=iroitemp;
            cout<<imageROIS->size()<<endl;
            vector<Point2i>*offstemp= new vector<Point2i>(numberOfTrackedColors);
            delete offsets;
            offsets=offstemp;
            cout<<offsets->size()<<endl;
            for(int i=0; i<numberOfTrackedColors; i++)
            {
                imageROIS->at(i)= Rect(0,0,640,480);
                offsets->at(i)= Point2i(0,0);
            }
            break;
        }
        case 'n': {
            cout<<"*Aborting..."<<endl;
            return 0;
        }
        }

    }

    }
    return 0;
}
//Function suppies breakpoint feature to wait for master system message
void waitforTrigger(boost::asio::ip::udp::udp::socket &socket)
{
    using boost::asio::ip::udp;

    char buffer [1024];
    udp::endpoint endpoint;
    boost::system::error_code error;
    socket.receive_from(boost::asio::buffer(buffer),endpoint,0,error);
    if(error && error != boost::asio::error::message_size)
    {
        throw boost::system::system_error(error);
    }
    cout<<"signal recieved"<<endl;
}

//Main Programm
int main()
{
    //TODO ADD PAPRAM FOR CONFIG FILE SELECTION (OPT.)
    cout<<"****RHOT Realtime Hand and Object Tracker v0.1****"<<endl;
    cout<<"*"<<endl;
    cout<<"*System is setting up starting values..."<<endl;
    cout<<"*"<<endl;
    boost::asio::io_service udp_io_service;
    boost::asio::io_service udp_sync_io_service;
    //DECOMMENT FOR MULTITHREAD WORKFLOW
    //  boost::asio::io_service ioService;
    //typedef boost::packaged_task<void> task_t;

    UDPClient client(udp_io_service,host,port);
    //setup UDP Sync Port
    using boost::asio::ip::udp;
    boost::asio::socket_base::reuse_address option(true);
    udp::socket masterSocket(udp_sync_io_service,udp::v4());
    masterSocket.set_option(option);
    masterSocket.bind(udp::endpoint(udp::v4(),6666));
    //DECOMMENT IF STEREO RECTIFICATION SHOULD BE DONE
    //readAndSetStereoCalibValues("intrinsics.yml", "extrinsics.yml", Size(frame_width,frame_height));

    //Setup Camera
    Camera->set(CV_CAP_PROP_FORMAT, CV_8UC3);
    Camera->set(CV_CAP_PROP_FRAME_WIDTH,frame_width);
    Camera->set(CV_CAP_PROP_FRAME_HEIGHT,frame_height);

    //Stop camera color drift by setting fixed values to gain and wb
    readCameraJSONAndSetValues(Camera,"cameraCalib.json");
    //Initialize Image ROI array values
    for(unsigned int i=0; i<imageROIS->size(); i++)
    {
        imageROIS->at(i)= Rect(0,0,640,480);
        offsets->at(i)= Point2i(0,0);
    }

    //DECOMMENT IF CIRCULAR BUFFER SHOULD BE USED
    //circular_buffer<Mat> *img_buffer= new circular_buffer<Mat>(5);

    //Default init
    cout<<"*The system loads color data from the colorCalibration.json for tracking colors.\n You can also switch to color calibration mode which does require a monitor to be conected."<<endl;
    cout<<"*"<<endl;
    cout<<"*Do you want to load colorCalibration.json values? (y/n)  ";
    char response;
    cin>>response;
    startPrompt(response);

    if ( !Camera->open())
    {
        cout<<"*"<<endl;
        cout << "Cannot open the web cam" << endl;
        return -1;
    }
    /*DECOMMENT FOR THREAD POOL INIT
        cout<<"*\n";
        cout<<"*Starting up Threadpool..."<<endl;
        boost::asio::io_service::work work(ioService);
        for(unsigned int i=0; i<2; i++)
        {
            threadpool.create_thread(boost::bind(&boost::asio::io_service::run,&ioService));
        }
        cout<<threadpool.size()<<endl;*/
    //  cout<<"*\n";
    //  cout<<"*Threadpool ready."<<endl;

    cout<<"*"<<endl;
    cout<<"*Camera warmup phase started"<<endl;
    sleep(5);
    cout<<"*"<<endl;
    cout<<"*Warmup finished"<<endl;
    //CAMERA THREAD
    //boost::thread cameraThread(getCameraFrame,img_buffer,Camera);
    //cout<<"waiting for buffer to fill"<<endl;
    //sleep(5);
    cout<<"*"<<endl;
    cout<<"*--->Starting processing"<<endl;
    cout<<"*"<<endl;
    cout<<"*System will be sending UDP Data to destination address: "<<host<<" to Port "<<port<<endl;
    //Setup wait for start signal
    cout<<"*"<<endl;
    cout<<"*System currently waiting for Master start signal..."<<endl;
    cout<<"*"<<endl;
    udp_io_service.run();
     //wait for Master to send start signal
    waitforTrigger(masterSocket);

    cout<<"Start signal recieved"<<endl;
    while(true) {
        t=clock();
        bool grabbed=Camera->grab();
        if(grabbed)
        {
            try {
                Camera->retrieve(*imgOriginal);
            } catch(...)
            {
                cout<<"*"<<endl;
                cout<<"*Camera retrieve error"<<endl;
            }

            //Application of image rectification for Stereo images
            //applySteroCalib(imgOriginal);

            //CAMERATHREAD + IMG_BUFFER
            //if(img_buffer->empty()==false)
            //{
            //Mat test=img_buffer->get().clone();
            //imgOriginal= &test;

            for(int k=0; k<numberOfTrackedColors; k++)
            {
                Mat cropped;
                try {
                    Mat cropped (*imgOriginal,imageROIS->at(k));
                    //SEQUENTIAL IMPL
                    color_detectThread((*colorThreshold)[k],cropped,(*contourCenters)[k],imageROIS->at(k),offsets->at(k),angle);
                    //MULTITHREAD IMPLEMENTAION
                    /*boost::shared_ptr<task_t> task= boost::make_shared<task_t>(
                                                        boost::bind(&color_detectThread,boost::ref((*colorThreshold)[k]),cropped,boost::ref((*contourCenters)[k]),boost::ref(imageROIS->at(k)),boost::ref(offsets->at(k)),angle));

                    boost::future<void> fut= task->get_future();
                    pending_data.push_back(std::move(fut));
                    ioService.post(boost::bind(&task_t::operator(),task));
                    //boost::asio::post(pool,boost::bind(&task_t::operator(),task));*/
                }
                catch(...)
                {
                    cout<<"*"<<endl;
                    cout<<imageROIS->at(k)<<endl;
                    cout<<"*Could not retrieve roi from sourceimage. taking default value."<<endl;
                    imageROIS->at(k)=Rect(0,0,frame_width,frame_height);
                }
            }
            //MULTITHREAD IMPLEMENTAION
            //cout<<"waiting for pool"<<endl;
            //boost::wait_for_all(pending_data.begin(),pending_data.end());
            //clock_t t1=clock()-t;

            for(int i=0; i<numberOfTrackedColors; i++)
            {
                //Write Datasets into stringstream for UDP Message
                ss<<((*contourCenters).at(i)+offsets->at(i))<<";";
                //DEBUG Visualization of tracking points and ROI area
                //cout<<((*contourCenters).at(i)+offsets->at(i))<<endl;
                //logger<<((*contourCenters).at(i)+offsets->at(i))<<";";
                //circle(*imgOriginal,((*contourCenters).at(i)+offsets->at(i)),5,Scalar(0,0,255),-1);
                //rectangle(*imgOriginal,imageROIS->at(i),Scalar(255,0,0));
                //DEBUG
            }
            //add calculated rotation angle to UDP Dataset
            ss<<angle<<";";
            //Get timestamp for current frame data
            using namespace std::chrono;
            {
                milliseconds ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
                ss<<ms.count()<<";";
            }
            client.send(ss.str());
            //clear string stream
            ss.str(string());
             //DEBUG
            //logger<<((float)t)/CLOCKS_PER_SEC<<"#\n";
            //TO DISPLAY DEBUG VIEW DECOMMENT
            //imshow("image",*imgOriginal);

        }
        else
        {
            cout<<"could not grab frame from camera"<<endl;
        }
        //TO DISPLAY DEGUB VIEW DECOMMENT
        //Wait for a keystroke in the window
        /*if((char)27==(char)waitKey(1)) {
            //delete img_buffer;
            //ioService.stop();
            //threadpool.join_all();
            destroyAllWindows();
            Camera->release();
            delete Camera;
            delete contourCenters;
            delete colorThreshold;
            delete imageROIS;
            delete offsets;

            return 0;
        }*/
        // count++;
        clock_t t2=clock()-t;
        printf("%f seconds\n",((float)t2)/CLOCKS_PER_SEC);
        //cout<<count<<endl;
    }
    //Logger for performance measurement
    //outfile<<logger.rdbuf();
    //outfile.close();
    Camera->release();
    //delete img_buffer;
    //ioService.stop();
    //threadpool.join_all();
    destroyAllWindows();
    delete Camera;
    delete contourCenters;
    delete colorThreshold;
    delete imageROIS;
    delete offsets;

    return 0;
}
