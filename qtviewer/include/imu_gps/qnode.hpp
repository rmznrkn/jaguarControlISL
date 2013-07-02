/**
 * @file /include/imu_gps/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef imu_gps_QNODE_HPP_
#define imu_gps_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QImage>
#include "imu.h"
#include "GPSFix.h"
#include "GPSStatus.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_plugin.h>
#include <compressed_image_transport/compressed_subscriber.h>
#include <compressed_image_transport/compression_common.h>
#include <opencv2/opencv.hpp>

#include <axis_camera/Axis.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace imu_gps {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

    ros::Publisher publisher_ptz_command;

Q_SIGNALS:

	void loggingUpdated();
    void rosShutdown();

    void imuSignal_accelX(QString);
    void imuSignal_accelY(QString);
    void imuSignal_accelZ(QString);

    void imuSignal_gyroX(QString);
    void imuSignal_gyroY(QString);
    void imuSignal_gyroZ(QString);

    void imuSignal_magnetomX(QString);
    void imuSignal_magnetomY(QString);
    void imuSignal_magnetomZ(QString);

    void imuSignal_raw(QString);

    void gpsSignal_raw(QString);
    void gpsSignal_latitude(QString);
    void gpsSignal_longtitude(QString);

    void gpsSignal_cog(QString);
    void gpsSignal_vog(QString);
    void gpsSignal_quality(QString);

    void scanSignal(const sensor_msgs::LaserScan);


    void frontCamera_signal(cv::Mat);
    void ptzCamera_signal(cv::Mat);
    void ptzStatus_signal(axis_camera::Axis);
private:
	int init_argc;
	char** init_argv;
    //ros::Publisher chatter_publisher;
    QStringListModel logging_model;

    ros::Subscriber subscriber_imu;
    ros::Subscriber subscriber_fix;
    ros::Subscriber subscriber_extendedfix;
    ros::Subscriber subscriber_scan;
    ros::Subscriber subscriber_ptzStatus;


    image_transport::Subscriber subscriber_frontCamera;
    image_transport::Subscriber subscriber_ptzCamera;
   // image_transport::
    void callback_gpsfix(const gps_node::GPSFixConstPtr &fix);
    void callback_nawsatfix(const sensor_msgs::NavSatFixConstPtr &fix);
    void callback_imu(const imu_node::imuConstPtr &imu_);
    void callback_scan(const sensor_msgs::LaserScanConstPtr& scan);

    void callback_frontCamera(const sensor_msgs::ImageConstPtr& msg);
    void callback_PTZCamera(const sensor_msgs::ImageConstPtr& msg);
    void callback_PTZStatus(axis_camera::Axis);

};
}  // namespace imu_gps
#endif /* imu_gps_QNODE_HPP_ */
