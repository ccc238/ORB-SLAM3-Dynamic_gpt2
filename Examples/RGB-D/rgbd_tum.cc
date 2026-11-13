/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include<System.h>
#include "SemanticMasker.h"

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int Run(int argc, char **argv)
{
    if(argc != 5 && argc != 6 && argc != 7)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association [output_directory] [max_frames]" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    cv::FileStorage runtimeSettings(argv[2],cv::FileStorage::READ);
    int imageIsRGB=0;
    if(!runtimeSettings["Camera.RGB"].empty()) runtimeSettings["Camera.RGB"] >> imageIsRGB;
    int useViewer=1;
    if(!runtimeSettings["Run.Viewer"].empty()) runtimeSettings["Run.Viewer"] >> useViewer;
    std::string output=argc>=6 ? argv[5] : ".";
    if(argc>=6) ORB_SLAM3::EnsureSemanticDirectory(output);
    if(argc==7) {
        int limit=std::stoi(argv[6]);
        if(limit<=0) throw std::runtime_error("max_frames must be positive");
        nImages=std::min(nImages,limit);
    }
    // The System constructor validates types and dependencies before any frames run.
    const auto filterSwitch=runtimeSettings["Dynamic.UseDynamicFilter"];
    const bool filterEnabled=filterSwitch.isInt() && int(filterSwitch)==0;
    std::string filterMode="simulate";
    if(runtimeSettings["Dynamic.FilterMode"].isString()) filterMode=std::string(runtimeSettings["Dynamic.FilterMode"]);
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,useViewer!=0);
    float imageScale = SLAM.GetImageScale();

    std::ofstream frameLog(output+"/RunFrames.csv");
    if(!frameLog) throw std::runtime_error("Cannot open RunFrames.csv");
    frameLog << "frame,timestamp,track_ms,keypoints,state,filter_enabled,filter_mode\n";
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty() || imD.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        // imread returns BGR/BGRA. Honor Camera.RGB before feeding both SLAM
        // and the semantic module, including the original TUM configurations.
        if(imageIsRGB) {
            if(imRGB.channels()==3) cv::cvtColor(imRGB,imRGB,cv::COLOR_BGR2RGB);
            else if(imRGB.channels()==4) cv::cvtColor(imRGB,imRGB,cv::COLOR_BGRA2RGBA);
        }

        if(imageScale != 1.f)
        {
            int width = imRGB.cols * imageScale;
            int height = imRGB.rows * imageScale;
            cv::resize(imRGB, imRGB, cv::Size(width, height));
            cv::resize(imD, imD, cv::Size(width, height));
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        try { SLAM.TrackRGBD(imRGB,imD,tframe); }
        catch(...) { SLAM.Shutdown(); throw; }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;
        frameLog << ni << ',' << std::setprecision(16) << tframe << ',' << ttrack*1000 << ','
                 << SLAM.GetTrackedKeyPointsUn().size() << ',' << SLAM.GetTrackingState() << ',' << filterEnabled << ',' << filterMode << '\n';

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

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
    SLAM.SaveTrajectoryTUM(output+"/CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM(output+"/KeyFrameTrajectory.txt");   

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    if(!fAssociation.is_open()) throw std::runtime_error("Cannot open association file: "+strAssociationFilename);
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty() && s[0]!='#')
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            double depthTime;
            if(!(ss >> t >> sRGB >> depthTime >> sD))
                throw std::runtime_error("Malformed association line: "+s);
            vTimestamps.push_back(t);
            vstrImageFilenamesRGB.push_back(sRGB);
            vstrImageFilenamesD.push_back(sD);

        }
    }
}

int main(int argc,char** argv) {
    try { return Run(argc,argv); }
    catch(const std::exception& e) { std::cerr << "RGB-D fatal error: " << e.what() << std::endl; return 2; }
}
