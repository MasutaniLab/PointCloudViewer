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
  cout << "PointCloudViewer::onInitialize()" << endl;
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
  cout << "PointCloudViewer::onActivated()" << endl;
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
  cout << "PointCloudViewer::onDeactivated()" << endl;
  m_viewer->close();
  return RTC::RTC_OK;
}

#define print(x) cout << #x << ": " << x << endl
#define printPointField(x) cout << #x << ": " << x.name << "," << x.offset << "," << x.data_type << "," << x.count << endl
#define printPtr(p) cout << #p << ": " << p.x << "," << p.y << "," << p.z << "," << int(p.r) << "," << int(p.g) << "," << int(p.b) << endl

RTC::ReturnCode_t PointCloudViewer::onExecute(RTC::UniqueId ec_id)
{
  if (m_pcIn.isNew())
  {
    m_pcIn.read();
    if (m_first) {
      print(m_pc.seq);
      print(m_pc.height);
      print(m_pc.width);
      print(m_pc.type);
      print(m_pc.is_bigendian);
      print(m_pc.point_step);
      print(m_pc.row_step);
      print(m_pc.is_dense);
      for (int i=0; i<m_pc.fields.length(); i++) {
        cout << i << " ";
        printPointField(m_pc.fields[i]);
      }
      m_first = false;
    }
    try {
    string type = m_pc.type;
    if (type == "xyz") {
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl_cloud->points.resize(m_pc.width*m_pc.height);
      float *src = (float *)m_pc.data.get_buffer();
      for (size_t i=0; i<pcl_cloud->points.size(); i++){
        pcl_cloud->points[i].x = src[0];
        pcl_cloud->points[i].y = src[1];
        pcl_cloud->points[i].z = src[2];
        src += 4;
      }
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr c(pcl_cloud);
      m_viewer->removePointCloud();
      m_viewer->addPointCloud<pcl::PointXYZ>(c);
    } else if (type == "xyzrgb") {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl_cloud->points.resize(m_pc.width*m_pc.height);
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
      pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr c(pcl_cloud);
      m_viewer->removePointCloud();
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(c);
      m_viewer->addPointCloud<pcl::PointXYZRGB>(c, rgb);
      //printPtr(pcl_cloud->points[0]);
      //printPtr(pcl_cloud->points[38399]);
      //printPtr(pcl_cloud->points[76799]);
    } else if (type == "xyzrgba") {
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl_cloud->points.resize(m_pc.width*m_pc.height);
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
      pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr c(pcl_cloud);
      m_viewer->removePointCloud();
      pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgba(c);
      m_viewer->addPointCloud<pcl::PointXYZRGBA>(c, rgba);
    } else {
      cerr << type << ": not supported" << endl;
      return RTC::RTC_ERROR;
    }
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
    //�_��\�����̑傫���i�P�ʁH�j
    m_viewer->spinOnce();
    if (m_viewer->wasStopped())
    {
        // Deactivate self
    }
    } catch ( std::exception e) {
      cerr << "catch: " << e.what();
      return RTC::RTC_ERROR;
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

