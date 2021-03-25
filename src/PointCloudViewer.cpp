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
using namespace Eigen;

// Module specification
// <rtc-template block="module_spec">
static const char* pointcloudviewer_spec[] =
  {
    "implementation_id", "PointCloudViewer",
    "type_name",         "PointCloudViewer",
    "description",       "Simple RTC:PCL Viewer",
    "version",           "1.0.0",
    "vendor",            "MasutaniLab",
    "category",          "RTCPCL",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.rotX", "0.0",
    "conf.default.rotY", "0.0",
    "conf.default.rotZ", "0.0",
    "conf.default.transX", "0.0",
    "conf.default.transY", "0.0",
    "conf.default.transZ", "0.0",
    "conf.default.colorOrder", "RGB",

    // Widget
    "conf.__widget__.rotX", "text",
    "conf.__widget__.rotY", "text",
    "conf.__widget__.rotZ", "text",
    "conf.__widget__.transX", "text",
    "conf.__widget__.transY", "text",
    "conf.__widget__.transZ", "text",
    "conf.__widget__.colorOrder", "radio",
    // Constraints
    "conf.__constraints__.colorOrder", "(RGB,BGR)",

    "conf.__type__.rotX", "double",
    "conf.__type__.rotY", "double",
    "conf.__type__.rotZ", "double",
    "conf.__type__.transX", "double",
    "conf.__type__.transY", "double",
    "conf.__type__.transZ", "double",
    "conf.__type__.colorOrder", "string",

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
  // Bind variables and configuration variable
  bindParameter("rotX", m_rotX, "0.0");
  bindParameter("rotY", m_rotY, "0.0");
  bindParameter("rotZ", m_rotZ, "0.0");
  bindParameter("transX", m_transX, "0.0");
  bindParameter("transY", m_transY, "0.0");
  bindParameter("transZ", m_transZ, "0.0");
  bindParameter("colorOrder", m_colorOrder, "RGB");
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
  double radX = m_rotX*M_PI / 180;
  double radY = m_rotY*M_PI / 180;
  double radZ = m_rotZ*M_PI / 180;
  Affine3f transform
    = Translation3f(m_transX, m_transY, m_transZ)
    *AngleAxisf(radZ, Vector3f::UnitZ())
    *AngleAxisf(radY, Vector3f::UnitY())
    *AngleAxisf(radX, Vector3f::UnitX());
  //cout << "m_transform:" << endl << m_transform.matrix() << endl;
  m_viewer.reset(new pcl::visualization::PCLVisualizer("PointCloudViewer"));
  m_viewer->setBackgroundColor(0, 0, 0);
  m_viewer->addCoordinateSystem(1.0);
  Vector3f origin = transform.translation();
  Vector3f camera = transform*Vector3f(0, 0, 3);
  Vector3f up = transform.linear().col(1);
  m_viewer->setCameraPosition(
    camera.x(), camera.y(), camera.z(),
    origin.x(), origin.y(), origin.z(),
    up.x(), up.y(), up.z());
#if 0
#define print(x) {cout << #x ":" << endl << x << endl;}
  print(camera);
  print(origin);
  print(up);
  print(transform.matrix());
#endif
  m_viewer->setCameraClipDistances(0,10);
  m_viewer->addCoordinateSystem(0.5, transform, "camera");

  //各画素のRとBを交換するかどうか（画素ごとの条件分岐を避けるために）
  if (m_colorOrder == "RGB") {
    m_setColor = m_swapRB;
  } else if (m_colorOrder == "BGR") {
    m_setColor = m_through;
  } else {
    RTC_ERROR(("m_colorOrder: %s は想定外", m_colorOrder.c_str()));
    return RTC::RTC_ERROR;
  }

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
  try {
    if (m_pcIn.isNew()) {
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
        for (size_t i = 0; i < m_pc.fields.length(); i++) {
          RTC_INFO_POINT_FIELD(i, m_pc.fields[i]);
        }
        m_first = false;
      }
      string type = string(m_pc.type);
      if (type == "xyz") {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_cloud->is_dense = m_pc.is_dense;
        pcl_cloud->points.resize(m_pc.width * m_pc.height);
        float* src = (float*)m_pc.data.get_buffer();
        for (size_t i = 0; i < pcl_cloud->points.size(); i++) {
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
        pcl_cloud->points.resize(m_pc.width * m_pc.height);
        pcl_cloud->is_dense = m_pc.is_dense;
        float* src = (float*)m_pc.data.get_buffer();
        for (size_t i = 0; i < pcl_cloud->points.size(); i++) {
          pcl_cloud->points[i].x = src[0];
          pcl_cloud->points[i].y = src[1];
          pcl_cloud->points[i].z = src[2];
          pcl_cloud->points[i].rgb = m_setColor(src[3]);
#if 0
          int x = i % m_pc.width;
          int y = i / m_pc.width;
          cout << i << "  " << x << "," << y << "  "
            << pcl_cloud->points[i].x << "," << pcl_cloud->points[i].y << "," << pcl_cloud->points[i].z << "  "
            << setfill('0') << setw(8) << right << hex << *(uint32_t*)&pcl_cloud->points[i].rgb << dec << endl;
#endif
#if 0
          int x = i % m_pc.width;
          int y = i / m_pc.width;
          if (x == m_pc.width / 2) {
            cout << i << "  " << x << "," << y << "  "
              << pcl_cloud->points[i].x << "," << pcl_cloud->points[i].y << "," << pcl_cloud->points[i].z << "  "
              << setfill('0') << setw(8) << right << hex << *(uint32_t*)&pcl_cloud->points[i].rgb << dec << endl;
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
        pcl_cloud->points.resize(m_pc.width * m_pc.height);
        pcl_cloud->is_dense = m_pc.is_dense;
        float* src = (float*)m_pc.data.get_buffer();
        for (size_t i = 0; i < pcl_cloud->points.size(); i++) {
          pcl_cloud->points[i].x = src[0];
          pcl_cloud->points[i].y = src[1];
          pcl_cloud->points[i].z = src[2];
          pcl_cloud->points[i].rgb = m_setColor(src[3]);
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
    }
    m_viewer->spinOnce(10);
    if (m_viewer->wasStopped()) {
        // Deactivate self
    }
  } catch (const std::length_error& e) {
    string w = e.what();
    if (w == "vector too long") {
      //pcl::visualization::PCLVisualizerInteractorStyle::OnTimer () 内の
      //Interactor->Render ()で発生する例外を回避するための苦肉の策
      RTC_WARN(("例外発生: %s (無視)", e.what()));
      return RTC::RTC_OK;
    }
    RTC_ERROR(("例外発生: %s", e.what()));
    return RTC::RTC_ERROR;
  } catch (const std::exception& e) {
    RTC_ERROR(("例外発生: %s", e.what()));
    return RTC::RTC_ERROR;
  } catch (...) {
    RTC_ERROR(("onExecute()において非標準の例外発生"));
    return RTC::RTC_ERROR;
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


