#include <iostream>
#include <math.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <vector>
#include <functional>
#include <boost/filesystem.hpp>
#include <pcl/io/ply_io.h>

#include "pcl_viewer.h"
#include "pcl_filter.h"

using namespace V3D;
using namespace std;

// use static mutex to lock thread when operating the pcl visuliazer
static std::mutex mtx;

void PrintHelp();
int  ParseArguments(int argc, char** argv,std::string& dir,double& resolution);
void UpdatePointCloud(PCLViewer* viewer, const std::string& dir, double resolution);

int main(int argc, char** argv) try
{
  std::string dir = "";
  double resolution = -1.0;
  int task = ParseArguments(argc,argv,dir,resolution);

  PCLViewer viewer("3D Point Cloud Viewer");
  if(task == 1)
  {
    std::thread t(UpdatePointCloud, &viewer, dir, resolution);
    while (!viewer.IsStop()) {
      mtx.lock();
      viewer.SpinOnce();
      mtx.unlock();
    }
    t.join();
  }
  else
  {
    PrintHelp();
  }

  viewer.SavePointCloud(viewer.PointCloud(),dir);
  return task;
}
catch (const std::exception& e)
{
  std::cout << e.what() << std::endl;
  return -1;
}

void UpdatePointCloud(PCLViewer* viewer, const std::string& dir, double resolution)
{
  // add a new thread for spin the viewer
  //std::thread t(ViewerSpin, viewer);
  std::vector<std::string> files;
  WSPointCloudPtr cloud(new WSPointCloud());
  PCLFilter filter;

  int all = 0;

  while (!viewer->IsStop())
  {
    std::vector<std::string> allfiles;
    boost::filesystem::directory_iterator itr(dir);
    for (; itr != boost::filesystem::directory_iterator(); ++itr)
    {
      if (boost::filesystem::is_regular_file(itr->status()));
        allfiles.push_back(itr->path().string());
    }
    all = allfiles.size();
    bool bNewCloud = false;
    for (const auto& file : allfiles)
    {
        int loaded = files.size();
        std::vector<std::string>::iterator end = files.end();
        if (std::find(files.begin(),files.end(),file) == files.end())
        {
            bNewCloud = true;
            files.push_back(file);
            std::cout << "pcl == load " << loaded << "th point cloud " << file << std::endl;
            WSPointCloudPtr temp(new WSPointCloud());
            int res = pcl::io::loadPLYFile(file, *temp);
            if(res < 0)
            {
              std::cout << "pcl == failed to load point cloud." << std::endl;
              continue;
            }

            mtx.lock();
            *cloud += *temp;
            mtx.unlock();
        }
     }

     if (bNewCloud)
     {
       mtx.lock();
       viewer->AddPointCloud(cloud);
       mtx.unlock();
     }

     // Sleep for 2 seconds for the ply file
     std::this_thread::sleep_for(std::chrono::seconds(2));
  }
}

void PrintHelp()
{
  std::cout << "This app is used for visualizing the point cloud by loading the '.ply' files in a directory.\n";
  std::cout << "Command line with providing a directory containing the .ply files:\n";
  std::cout << "    view point cloud:   cmd [-view|-v] [file_directory] [octree resolution]\n";
}

int ParseArguments(int argc, char** argv, std::string& dir, double& resolution)
{
  if (argc < 2)
    return 0;
  std::string task = std::string(argv[1]);
  if (task.compare("-v") == 0 || task.compare("-view") == 0)
  {
    dir = std::string(argv[2]);
    if (argc > 3)
      resolution = std::stod(argv[3]);
    return 1;
  }
}
