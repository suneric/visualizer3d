#ifndef _ASV3D_PCL_VIEWER_H_
#define _ASV3D_PCL_VIEWER_H_

#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

typedef pcl::PointXYZRGB WSPoint;
typedef pcl::PointCloud<WSPoint> WSPointCloud;
typedef WSPointCloud::Ptr WSPointCloudPtr;

typedef pcl::Normal WSNormal;
typedef pcl::PointCloud<WSNormal> WSPointCloudNormal;
typedef WSPointCloudNormal::Ptr WSPointCloudNormalPtr;

typedef pcl::PointXYZRGBNormal WSNormalPoint;
typedef pcl::PointCloud<WSNormalPoint> WSNormalPointCloud;
typedef WSNormalPointCloud::Ptr WSNormalPointCloudPtr;

namespace V3D
{
  class PCLViewer
  {
  public:
    PCLViewer(const std::string& title);
    ~PCLViewer();

    WSPointCloudPtr PointCloud();
    WSPointCloudPtr LoadPointCloud(const std::string& dir);
    bool SavePointCloud(const WSPointCloudPtr cloud, const std::string& dir);

    int CreateViewPort(double xmin,double ymin,double xmax,double ymax);

    void AddPointCloud(const WSPointCloudPtr cloud, int viewport=0);
    void AddNormals(const WSPointCloudPtr cloud, const WSPointCloudNormalPtr normal, int size, double arrow, int viewport=0);
    void AddMesh(const pcl::PolygonMesh& mesh);
    void AddArrow(const WSPoint& startPt, const WSPoint& endPt, const std::string& name, int viewport=0);
    void AddArrows(const WSPointCloudPtr cloud, const WSPointCloudNormalPtr normal, double length, int viewport=0);
    void AddCoordinateSystem(const Eigen::Affine3f& camPose, int idIndex, int viewport=0, bool removeall=false);
    void AddCube(const WSPoint& point, double s, const std::string& id, double r,double g, double b, int viewport=0);
    void AddText(const std::string& text, const std::string& id, int viewport=0);
    void AddLine(const WSPoint& startPt, const WSPoint& endPt, int idx, double r,double g, double b, int viewport=0);
    void AddPolygon(const WSPointCloudPtr& polygon, const std::string& id, double r, double g, double b, int viewport=0);


    bool IsStop() const;
    void SpinOnce(double duration = 1);
    void Spin() const;

  private:
    pcl::visualization::PCLVisualizer* m_viewer;
    WSPointCloudPtr m_ptCloud;
  };

};

#endif //!_ASV3D_PCL_VIEWER_H_
