#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<string>
#include<unistd.h>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<System.h>
#include<cstdlib>
enum Datatype{videodata=0, imagesdata=1};
int datatype =0; 

using namespace std;
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);
void writeVideoInfo(vector<string> &vstrImageFilenames, vector<double> &vTimestamps, int TotalFrameNumber);

int main(int argc, char **argv)
{
    // if(argc != 4)
    // {
    //     cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
    //     cerr << endl << "or ./mono_tum path_to_vocabulary path_to_settings path_to_video" << endl;
    //     return 1;
    // // }
     argv[1] = "../../Vocabulary/ORBvoc.txt";
     argv[2] = "TUM1.yaml";
     argv[3] = "../../rgbd_dataset_freiburg1_xyz";

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    std::string dataname = argv[3];
    cv::VideoCapture cap(dataname);

    //choose datatype by isOpened()
    if(cap.isOpened())
    {    
        datatype = Datatype::videodata;
    int TotalFrameNumber = cap.get(CV_CAP_PROP_FRAME_COUNT);
    writeVideoInfo(vstrImageFilenames, vTimestamps, TotalFrameNumber);
    }
    else
    {
        datatype = Datatype::imagesdata;
    std::string strFile = string(argv[3])+"/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);
    }
    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        double tframe = vTimestamps[ni];
        if(datatype == Datatype::videodata)
        {
        cap.read(im);
        std::string nframe = vstrImageFilenames[ni];
        cv::imwrite("DataforORBSLAM/rgb/"+nframe, im);
        }
        else if(datatype == Datatype::imagesdata)
        {
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],cv::IMREAD_UNCHANGED);
        }

        
        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
        if(datatype == Datatype::videodata)
        {
        im+=1;
        }
    }
    //get image size
    cv::Size s = im.size();
    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.ngp_json("transforms.json", s);
    // SLAM.SaveMap("sfm.txt",im.size );


    return 0;
}


void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    // 前三行是注释，跳过
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}

//to generate video info file
void writeVideoInfo(vector<string> &vstrImageFilenames, vector<double> &vTimestamps, int TotalFrameNumber)
{
    
    system("mkdir -p DataforORBSLAM/rgb/");
    std::ofstream outtofile;
    std::string FilePath = "DataforORBSLAM/rgb.txt";
    outtofile.open(FilePath); 
    double beginname = 6269.100000;
    std::string filename;
    for(int i=0; i<TotalFrameNumber; i++)
    {
        std::string str = std::to_string(beginname)  +".png";
        vstrImageFilenames.push_back(str);
        vTimestamps.push_back(beginname);
        outtofile <<std::to_string(beginname)<<"   rgb/"+str<< std::endl;
        beginname+=0.000001;
    }
    outtofile.close();
    std::cout<<"write txt done"<<std::endl;
}