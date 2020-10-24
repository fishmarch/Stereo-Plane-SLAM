//
// Created by fishmarch on 20-6-22.
//

#include "Config.h"

namespace ORB_SLAM2{
    void Config::SetParameterFile( const std::string& filename )
    {
        if ( mConfig == nullptr )
            mConfig = shared_ptr<Config>(new Config);
        mConfig->mFile = cv::FileStorage( filename.c_str(), cv::FileStorage::READ );
        if ( !mConfig->mFile.isOpened())
        {
            std::cerr<<"parameter file "<< filename <<" does not exist."<<std::endl;
            mConfig->mFile.release();
            return;
        }
    }

    Config::~Config()
    {
        if ( mFile.isOpened() )
            mFile.release();
    }

    shared_ptr<Config> Config::mConfig = nullptr;
}
