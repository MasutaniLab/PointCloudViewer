// -*- C++ -*-
/*!
 * @file  PointCloudViewer.cpp
 * @brief Simple RTC:PCL Viewer
 * @date $Date$
 *
 * $Id$
 */

#include "PointCloudViewer.h"

#include <iostream>
#include <sstream>
using namespace std;

// Module specification
// <rtc-template block="module_spec">
static const char* pointcloudviewer_spec[] =
  {
    "implementation_id", "PointCloudViewer",
    "type_name",         "PointCloudViewer",
    "description",       "Simple RTC:PCL Viewer",
    "version",           "1.0.0",
    "vendor",            "OECU",
    "category",          "RTCPCL",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
PointCloudViewer::PointCloudViewer(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_pcIn("pc", m_pc)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
PointCloudViewer::~PointCloudViewer()
{
}



RTC::ReturnCode_t PointCloudViewer::onInitialize()
{
  RTC_INFO(("onInitialize()"));
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("pc", m_pcIn);
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t PointCloudViewer::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PointCloudViewer::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PointCloudViewer::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t PointCloudViewer::onActivated(RTC::UniqueId ec_id)
{
  RTC_INFO(("onActivated()"));
  m_viewer.reset(new pcl::visualization::PCLVisualizer("PointCloudViewer"));
  m_viewer->setBackgroundColor(0, 0, 0);
  m_viewer->addCoordinateSystem(1.0);
  m_viewer->setCameraPosition(0,0,3, 0,0,0, 0,1,0);
  m_viewer->setCameraClipDistances(0,10);
  m_first = true;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t PointCloudViewer::onDeactivated(RTC::UniqueId ec_id)
{
  RTC_INFO(("onDeactivated()"));
  m_viewer->close();
  return RTC::RTC_OK;
}

#define RTC_INFO_VAR(x) {stringstream ss; ss << #x << ": " << x; RTC_INFO(((ss.str().c_str())))}
#define RTC_INFO_POINT_FIELD(i, x) \
  {stringstream ss; \
   ss << i << " " << #x << ": " << x.name << "," << x.offset << "," << x.data_type << "," << x.count;\
   RTC_INFO(((ss.str().c_str())))}

RTC::ReturnCode_t PointCloudViewer::onExecute(RTC::UniqueId ec_id)
{
  if (m_pcIn.isNew())
  {
    m_pcIn.read();
    if (m_first) {
      RTC_INFO_VAR(m_pc.seq);
      RTC_INFO_VAR(m_pc.height);
      RTC_INFO_VAR(m_pc.width);
      RTC_INFO_VAR(m_pc.type);
      RTC_INFO_VAR(m_pc.is_bigendian);
      RTC_INFO_VAR(m_pc.point_step);
      RTC_INFO_VAR(m_pc.row_step);
      RTC_INFO_VAR(m_pc.is_dense);
      for (int i=0; i<m_pc.fields.length(); i++) {
        RTC_INFO_POINT_FIELD(i, m_pc.fields[i]);
      }
      m_first = false;
    }
    string type = m_pc.type;
    if (type == "xyz") {
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl_cloud->is_dense = m_pc.is_dense;
      pcl_cloud->points.resize(m_pc.width*m_pc.height);
      float *src = (float *)m_pc.data.get_buffer();
      for (size_t i=0; i<pcl_cloud->points.size(); i++){
        pcl_cloud->points[i].x = src[0];
        pcl_cloud->points[i].y = src[1];
        pcl_cloud->points[i].z = src[2];
        src += 4;
      }
      if (!m_viewer->updatePointCloud(pcl_cloud, "cloud")) {
        m_viewer->addPointCloud<pcl::PointXYZ>(pcl_cloud, "cloud");
        m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
      }
    } else if (type == "xyzrgb") {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl_cloud->points.resize(m_pc.width*m_pc.height);
      pcl_cloud->is_dense = m_pc.is_dense;
      float *src = (float *)m_pc.data.get_buffer();
      for (size_t i=0; i<pcl_cloud->points.size(); i++){
        pcl_cloud->points[i].x = src[0];
        pcl_cloud->points[i].y = src[1];
        pcl_cloud->points[i].z = src[2];
        pcl_cloud->points[i].rgb = src[3];
#ifdef SWAP_R_B      
        uint8_t t = pcl_cloud->points[i].r;
        pcl_cloud->points[i].r = pcl_cloud->points[i].b;
        pcl_cloud->points[i].b = t;
#endif
#if 0
        int x = i%m_pc.width;
        int y = i/m_pc.width; 
        cout << i << "  " << x << "," << y << "  "
          << pcl_cloud->points[i].x << "," << pcl_cloud->points[i].y << "," << pcl_cloud->points[i].z << "  "
          <<  setfill('0') << setw(8) << right << hex << *(uint32_t *)&pcl_cloud->points[i].rgb << dec << endl;
#endif
#if 0
        int x = i%m_pc.width;
        int y = i/m_pc.width;
        if ( x == m_pc.width/2) {
          cout << i << "  " << x << "," << y << "  "
            << pcl_cloud->points[i].x << "," << pcl_cloud->points[i].y << "," << pcl_cloud->points[i].z << "  "
            <<  setfill('0') << setw(8) << right << hex << *(uint32_t *)&pcl_cloud->points[i].rgb << dec << endl;
        }
#endif
        src += 4;
      }
      if (!m_viewer->updatePointCloud(pcl_cloud, "cloud")) {
        m_viewer->addPointCloud<pcl::PointXYZRGB>(pcl_cloud, "cloud");
        m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
      }
    } else if (type == "xyzrgba") {
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl_cloud->points.resize(m_pc.width*m_pc.height);
      pcl_cloud->is_dense = m_pc.is_dense;
      float *src = (float *)m_pc.data.get_buffer();
      for (size_t i=0; i<pcl_cloud->points.size(); i++){
        pcl_cloud->points[i].x = src[0];
        pcl_cloud->points[i].y = src[1];
        pcl_cloud->points[i].z = src[2];
        pcl_cloud->points[i].rgb = src[3];
#ifdef SWAP_R_B      
        uint8_t t = pcl_cloud->points[i].r;
        pcl_cloud->points[i].r = pcl_cloud->points[i].b;
        pcl_cloud->points[i].b = t;
#endif
        src += 4;
      }
      if (!m_viewer->updatePointCloud(pcl_cloud, "cloud")) {
        m_viewer->addPointCloud<pcl::PointXYZRGBA>(pcl_cloud, "cloud");
        m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
      }
    } else {
      RTC_ERROR(((type + ": not supported").c_str()));
      return RTC::RTC_ERROR;
    }
    m_viewer->spinOnce();
    if (m_viewer->wasStopped())
    {
        // Deactivate self
    }
  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t PointCloudViewer::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PointCloudViewer::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PointCloudViewer::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PointCloudViewer::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PointCloudViewer::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void PointCloudViewerInit(RTC::Manager* manager)
  {
    coil::Properties profile(pointcloudviewer_spec);
    manager->registerFactory(profile,
                             RTC::Create<PointCloudViewer>,
                             RTC::Delete<PointCloudViewer>);
  }
  
};


