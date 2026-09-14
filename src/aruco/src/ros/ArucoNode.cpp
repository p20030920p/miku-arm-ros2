#include <iostream>
#include <string>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo/photo.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// ROS 2 差异：ros/ros.h -> rclcpp/rclcpp.hpp
#include "rclcpp/rclcpp.hpp"

// ROS 2 差异：消息头文件全部带 msg 子目录，类型带 msg 命名空间
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "nav_msgs/msg/odometry.hpp"

// ROS 2 差异：Jazzy 中 cv_bridge 的头文件是 <cv_bridge/cv_bridge.hpp>
#include "cv_bridge/cv_bridge.hpp"

// ROS 2 差异：aruco/Marker.h -> aruco/msg/marker.hpp
#include "aruco/msg/marker.hpp"

// ROS 2 差异：不再使用 image_transport::ImageTransport + it.subscribe(...)（Jazzy 中该经典 API
// 已弃用且容易出错），改为直接用 rclcpp 订阅 sensor_msgs::msg::Image，在回调里用 cv_bridge 转换，
// 话题名与消息类型保持与 ROS 1 完全一致（见构造函数中的 sub_camera）。
// 代价是不再自动支持 /compressed 等 image_transport 传输插件话题，节点对外接口不变。

#include "../ArucoMarker.cpp"
#include "../ArucoMarkerInfo.cpp"
#include "../ArucoDetector.cpp"

using namespace cv;
using namespace std;

/**
 * Camera calibration matrix pre initialized with calibration values for the test camera.
 */
double data_calibration[9] = {570.3422241210938, 0, 319.5, 0, 570.3422241210938, 239.5, 0, 0, 1};

/**
 * Lenses distortion matrix initialized with values for the test camera.
 */
double data_distortion[5] = {0, 0, 0, 0, 0};

/**
 * ROS 2 差异：ROS 1 中 ros::NodeHandle node("aruco") 的 getNamespace() 返回 "/aruco"，
 * 发布话题名由 node.getNamespace() + 参数值 拼接而成（默认得到 /aruco/visible 等）。
 * ROS 2 的话题名按节点自身命名空间（"/"，节点名为 "aruco"）解析，若直接用参数值会得到
 * /visible、/position…… 因此这里保留 ROS 1 的命名空间常量，显式复现原来的拼接规则，
 * 保证外部话题契约（/aruco/visible、/aruco/position、/aruco/rotation、/aruco/pose、
 * /aruco/odom）完全不变。
 */
const string node_namespace = "/aruco";

/**
 * ROS 2 差异：复现 ROS 1 中 ros::NodeHandle("aruco") 对订阅话题名的解析规则
 * （ros::names::resolve("/aruco", name)）：绝对名保持不变，相对名挂到 "/aruco" 下，
 * 空字符串解析为命名空间本身。这样订阅话题与 ROS 1 保持一致。
 * @param name Topic name read from the node parameters.
 * @return Resolved topic name.
 */
string resolveName(const string& name)
{
	if(name.empty())
	{
		return node_namespace;
	}

	if(name[0] == '/')
	{
		return name;
	}

	return node_namespace + "/" + name;
}

/**
 * Draw yellow text with black outline into a frame.
 * @param frame Frame mat.
 * @param text Text to be drawn into the frame.
 * @param point Position of the text in frame coordinates.
 */
void drawText(Mat frame, string text, Point point)
{
	putText(frame, text, point, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 2, cv::LINE_AA);
	putText(frame, text, point, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1, cv::LINE_AA);
}

/**
 * Converts a string with numeric values separated by a delimiter to an array of double values.
 * If 0_1_2_3 and delimiter is _ array will contain {0, 1, 2, 3}.
 * @param data String to be converted
 * @param values Array to store values on
 * @param cout Number of elements in the string
 * @param delimiter Separator element
 * @return Array with values.
 */
void stringToDoubleArray(string data, double* values, unsigned int count, string delimiter)
{
	unsigned int pos = 0, k = 0;

	while((pos = data.find(delimiter)) != string::npos && k < count)
	{
		string token = data.substr(0, pos);
		values[k] = stod(token);
		data.erase(0, pos + delimiter.length());
		k++;
	}
}

/**
 * ROS 2 差异：ROS 1 中节点是一个带全局变量的 main()（ros::NodeHandle node("aruco")）；
 * ROS 2 中节点继承 rclcpp::Node，原来的全局变量改为成员变量，回调改为成员函数，
 * 参数读取改为节点参数（declare_parameter + get_parameter）。算法与话题接口完全不变。
 */
class ArucoNode : public rclcpp::Node
{
	public:
		/**
		 * @param options Node options, required to enable automatic parameter declaration
		 * (equivalent of the ROS 1 parameter server lookups done with nh.hasParam).
		 */
		ArucoNode(const rclcpp::NodeOptions& options) : rclcpp::Node("aruco", options)
		{
			//Parameters
			// ROS 2 差异：ros::NodeHandle::param<T>() -> 节点参数（declare_parameter + get_parameter）。
			// 本节点以 automatically_declare_parameters_from_overrides(true) 启动，参数文件/launch/
			// 命令行给出的参数已被自动声明（等价于 ROS 1 参数服务器里已存在的参数），
			// getParameter() 只在参数尚未声明时才用默认值声明，语义与 nh.param() 相同。
			debug = this->getParameter<bool>("debug", false);
			use_opencv_coords = this->getParameter<bool>("use_opencv_coords", false);

			// ROS 2 差异：ROS 1 的 float 参数在 ROS 2 中按 double 声明后窄化回 float，保持原数值语义
			cosine_limit = static_cast<float>(this->getParameter<double>("cosine_limit", 0.7));

			theshold_block_size_min = static_cast<int>(this->getParameter<int>("theshold_block_size_min", 3));
			theshold_block_size_max = static_cast<int>(this->getParameter<int>("theshold_block_size_max", 21));

			max_error_quad = static_cast<float>(this->getParameter<double>("max_error_quad", 0.035));

			min_area = static_cast<int>(this->getParameter<int>("min_area", 100));

			calibrated = this->getParameter<bool>("calibrated", false);

			//Initial threshold block size
			theshold_block_size = (theshold_block_size_min + theshold_block_size_max) / 2;
			if(theshold_block_size % 2 == 0)
			{
				theshold_block_size++;
			}

			//Initialize calibration matrices
			calibration = Mat(3, 3, CV_64F, data_calibration);
			distortion = Mat(1, 5, CV_64F, data_distortion);

			//Camera instrinsic calibration parameters
			// ROS 2 差异：ROS 1 用 nh.hasParam("calibration") 判断参数是否存在；ROS 2 的参数必须先声明，
			// 本节点以 automatically_declare_parameters_from_overrides(true) 启动（见 main），参数文件/
			// 命令行（launch）给出的参数会被自动声明，has_parameter() 即等价于原来的 hasParam()。
			if(this->has_parameter("calibration"))
			{
				string data = this->get_parameter("calibration").as_string();

				double values[9];
				stringToDoubleArray(data, values, 9, "_");

				for(unsigned int i = 0; i < 9; i++)
				{
					calibration.at<double>(i / 3, i % 3) = values[i];
				}

				calibrated = true;
			}

			//Camera distortion calibration parameters
			if(this->has_parameter("distortion"))
			{
				string data = this->get_parameter("distortion").as_string();

				double values[5];
				stringToDoubleArray(data, values, 5, "_");

				for(unsigned int i = 0; i < 5; i++)
				{
					distortion.at<double>(0, i) = values[i];
				}

				calibrated = true;
			}

			//Aruco makers passed as parameters
			for(unsigned int i = 0; i < 1024; i++)
			{
				if(this->has_parameter("marker" + to_string(i)))
				{
					string data = this->get_parameter("marker" + to_string(i)).as_string();

					double values[7];
					stringToDoubleArray(data, values, 7, "_");

					//Use OpenCV coordinates
					if(use_opencv_coords)
					{
						known.push_back(ArucoMarkerInfo(i, values[0], Point3d(values[1], values[2], values[3]), Point3d(values[4], values[5], values[6])));
					}
					//Convert coordinates (-Y, -Z, +X)
					else
					{
						known.push_back(ArucoMarkerInfo(i, values[0], Point3d(-values[2], -values[3], -values[1]), Point3d(-values[5], -values[6], values[4])));
					}
				}
			}

			//Print all known markers
			if(debug)
			{
				for(unsigned int i = 0; i < known.size(); i++)
				{
					known[i].print();
				}
			}

			// TF frame
			tf_frame_id = this->getParameter<std::string>("tf_frame_id", "robot");

			//Subscribed topic names
			string topic_camera, topic_camera_info, topic_marker_register, topic_marker_remove;
			topic_camera = this->getParameter<std::string>("topic_camera", "/rgb/image");
			topic_camera_info = this->getParameter<std::string>("topic_camera_info", "/rgb/camera_info");
			topic_marker_register = this->getParameter<std::string>("topic_marker_register", "/marker_register");

			// ROS 2 差异：ROS 1 源码此处有一处笔误——第二行把 "topic_marker_remove" 参数读进了
			// topic_marker_register 变量，使 topic_marker_remove 恒为空串（ROS 1 中空话题名解析为节点
			// 命名空间 "/aruco"），结果是标记注册订阅实际落在 /marker_remove 上、标记删除订阅失效。
			// ROS 2 不允许空话题名，这里按参数名的本意分别读取（参数名与默认值与 ROS 1 完全一致）：
			// 注册走 /marker_register，删除走 /marker_remove。
			topic_marker_remove = this->getParameter<std::string>("topic_marker_remove", "/marker_remove");

			//Publish topic names
			string topic_visible, topic_position, topic_rotation, topic_pose, topic_odom;
			topic_visible = this->getParameter<std::string>("topic_visible", "/visible");
			topic_position = this->getParameter<std::string>("topic_position", "/position");
			topic_rotation = this->getParameter<std::string>("topic_rotation", "/rotation");
			topic_pose = this->getParameter<std::string>("topic_pose", "/pose");
			topic_odom = this->getParameter<std::string>("topic_odom", "/odom");

			//Advertise topics
			// ROS 2 差异：nh.advertise<T>(...) -> create_publisher<T>(...)；
			// 话题名的拼接规则（node_namespace + 参数值）与 ROS 1 完全一致。
			pub_visible = this->create_publisher<std_msgs::msg::Bool>(node_namespace + topic_visible, 10);
			pub_position = this->create_publisher<geometry_msgs::msg::Point>(node_namespace + topic_position, 10);
			pub_rotation = this->create_publisher<geometry_msgs::msg::Point>(node_namespace + topic_rotation, 10);
			pub_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>(node_namespace + topic_pose, 10);
			pub_odom = this->create_publisher<nav_msgs::msg::Odometry>(node_namespace + topic_odom, 10);

			//Subscribe topics
			// ROS 2 差异：ROS 1 用 image_transport::ImageTransport::subscribe 订阅图像（队列深度 1）。
			// Jazzy 中该经典 API 已弃用，这里直接订阅 sensor_msgs::msg::Image，话题名与消息类型与
			// ROS 1 完全一致，队列深度同样为 1。
			// QoS 采用 SensorDataQoS（best effort）：ROS 2 的相机驱动普遍以 best effort 发布图像/
			// 内参，可靠的订阅者会与之 QoS 不兼容而收不到数据；best effort 订阅对 reliable 与
			// best_effort 两种发布者都兼容。
			sub_camera = this->create_subscription<sensor_msgs::msg::Image>(
				resolveName(topic_camera), rclcpp::SensorDataQoS().keep_last(1),
				std::bind(&ArucoNode::onFrame, this, std::placeholders::_1));

			// ROS 2 差异：相机内参订阅使用与图像一致的 SensorDataQoS（队列深度 1）
			sub_camera_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
				resolveName(topic_camera_info), rclcpp::SensorDataQoS().keep_last(1),
				std::bind(&ArucoNode::onCameraInfo, this, std::placeholders::_1));

			sub_marker_register = this->create_subscription<aruco::msg::Marker>(
				resolveName(topic_marker_register), 1,
				std::bind(&ArucoNode::onMarkerRegister, this, std::placeholders::_1));

			sub_marker_remove = this->create_subscription<std_msgs::msg::Int32>(
				resolveName(topic_marker_remove), 1,
				std::bind(&ArucoNode::onMarkerRemove, this, std::placeholders::_1));
		}

	private:
		/**
		 * ROS 2 差异：ROS 1 的 nh.param<T>(name, out, default) 在参数不存在时使用默认值。
		 * ROS 2 中参数必须先声明：本节点以 automatically_declare_parameters_from_overrides(true)
		 * 启动，参数文件/launch/命令行提供的参数已被自动声明（相当于 ROS 1 参数服务器里已存在的
		 * 参数），因此这里只在参数尚未声明时用默认值声明，然后读取——与 nh.param() 语义一致。
		 * @param name Parameter name.
		 * @param default_value Value used when the parameter was not provided.
		 * @return Parameter value.
		 */
		template<typename T>
		T getParameter(const std::string& name, const T& default_value)
		{
			if(!this->has_parameter(name))
			{
				this->declare_parameter<T>(name, default_value);
			}

			return this->get_parameter(name).get_value<T>();
		}

		/**
		 * Callback executed every time a new camera frame is received.
		 * This callback is used to process received images and publish messages with camera position data if any.
		 * ROS 2 差异：回调参数由 const sensor_msgs::ImageConstPtr& 改为 ConstSharedPtr（按值传递）。
		 */
		void onFrame(const sensor_msgs::msg::Image::ConstSharedPtr msg)
		{
			try
			{
				Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

				//Process image and get markers
				vector<ArucoMarker> markers = ArucoDetector::getMarkers(frame, cosine_limit, theshold_block_size, min_area, max_error_quad);

				//Visible
				vector<ArucoMarker> found;

				//Vector of points
				vector<Point2f> projected;
				vector<Point3f> world;

				if(markers.size() == 0)
				{
					theshold_block_size += 2;

					if(theshold_block_size > theshold_block_size_max)
					{
						theshold_block_size = theshold_block_size_min;
					}
				}

				//Check known markers and build known of points
				for(unsigned int i = 0; i < markers.size(); i++)
				{
					for(unsigned int j = 0; j < known.size(); j++)
					{
						if(markers[i].id == known[j].id)
						{
							markers[i].attachInfo(known[j]);

							for(unsigned int k = 0; k < 4; k++)
							{
								projected.push_back(markers[i].projected[k]);
								world.push_back(known[j].world[k]);
							}

							found.push_back(markers[i]);
						}
					}
				}

				//Draw markers
				if(debug)
				{
					ArucoDetector::drawMarkers(frame, markers, calibration, distortion);
				}

				//Check if any marker was found
				if(world.size() > 0)
				{
					//Calculate position and rotation
					Mat rotation, position;

					#if CV_MAJOR_VERSION == 2
						solvePnP(world, projected, calibration, distortion, rotation, position, false, ITERATIVE);
					#else
						solvePnP(world, projected, calibration, distortion, rotation, position, false, SOLVEPNP_ITERATIVE);
					#endif

					//Invert position and rotation to get camera coords
					Mat rodrigues;
					Rodrigues(rotation, rodrigues);

					Mat camera_rotation;
					Rodrigues(rodrigues.t(), camera_rotation);

					Mat camera_position = -rodrigues.t() * position;

					//Publish position and rotation
					geometry_msgs::msg::Point message_position, message_rotation;

					//OpenCV coordinates
					if(use_opencv_coords)
					{
						message_position.x = camera_position.at<double>(0, 0);
						message_position.y = camera_position.at<double>(1, 0);
						message_position.z = camera_position.at<double>(2, 0);

						message_rotation.x = camera_rotation.at<double>(0, 0);
						message_rotation.y = camera_rotation.at<double>(1, 0);
						message_rotation.z = camera_rotation.at<double>(2, 0);
					}
					//Robot coordinates
					else
					{
						message_position.x = camera_position.at<double>(2, 0);
						message_position.y = -camera_position.at<double>(0, 0);
						message_position.z = -camera_position.at<double>(1, 0);

						message_rotation.x = camera_rotation.at<double>(2, 0);
						message_rotation.y = -camera_rotation.at<double>(0, 0);
						message_rotation.z = -camera_rotation.at<double>(1, 0);
					}

					pub_position->publish(message_position);
					pub_rotation->publish(message_rotation);

					//Publish pose
					geometry_msgs::msg::PoseStamped message_pose;

					//Header
					message_pose.header.frame_id = tf_frame_id;
					// ROS 2 差异：ROS 2 的 std_msgs/Header 已删除 seq 字段，消息顺序由 stamp 标识；
					// 这里保留原来的计数器语义（pub_pose_seq 仍然递增，但不再写入 header）。
					pub_pose_seq++;
					message_pose.header.stamp = this->now();

					//Position
					message_pose.pose.position.x = message_position.x;
					message_pose.pose.position.y = message_position.y;
					message_pose.pose.position.z = message_position.z;

					//Convert to quaternion
					double x = message_rotation.x;
					double y = message_rotation.y;
					double z = message_rotation.z;

					//Module of angular velocity
					double angle = sqrt(x*x + y*y + z*z);
					if(angle > 0.0)
					{
						message_pose.pose.orientation.x = x * sin(angle/2.0)/angle;
						message_pose.pose.orientation.y = y * sin(angle/2.0)/angle;
						message_pose.pose.orientation.z = z * sin(angle/2.0)/angle;
						message_pose.pose.orientation.w = cos(angle/2.0);
					}
					//To avoid illegal expressions
					else
					{
						message_pose.pose.orientation.x = 0.0;
						message_pose.pose.orientation.y = 0.0;
						message_pose.pose.orientation.z = 0.0;
						message_pose.pose.orientation.w = 1.0;
					}

					pub_pose->publish(message_pose);

					nav_msgs::msg::Odometry message_odometry;
					message_odometry.header.frame_id = tf_frame_id;
					message_odometry.header.stamp = this->now();
					message_odometry.pose.pose = message_pose.pose;
					pub_odom->publish(message_odometry);

					//Debug
					if(debug)
					{
						ArucoDetector::drawOrigin(frame, found, calibration, distortion, 0.1);

						drawText(frame, "Position: " + to_string(message_position.x) + ", " + to_string(message_position.y) + ", " + to_string(message_position.z), Point2f(10, 180));
						drawText(frame, "Rotation: " + to_string(message_rotation.x) + ", " + to_string(message_rotation.y) + ", " + to_string(message_rotation.z), Point2f(10, 200));
					}
				}
				else if(debug)
				{
					drawText(frame, "Position: unknown", Point2f(10, 180));
					drawText(frame, "Rotation: unknown", Point2f(10, 200));
				}

				//Publish visible
				std_msgs::msg::Bool message_visible;
				message_visible.data = world.size() != 0;
				pub_visible->publish(message_visible);

				//Debug info
				if(debug)
				{
					drawText(frame, "Aruco ROS Debug", Point2f(10, 20));
					drawText(frame, "OpenCV V" + to_string(CV_MAJOR_VERSION) + "." + to_string(CV_MINOR_VERSION), Point2f(10, 40));
					drawText(frame, "Cosine Limit (A-Q): " + to_string(cosine_limit), Point2f(10, 60));
					drawText(frame, "Threshold Block (W-S): " + to_string(theshold_block_size), Point2f(10, 80));
					drawText(frame, "Min Area (E-D): " + to_string(min_area), Point2f(10, 100));
					drawText(frame, "MaxError PolyDP (R-F): " + to_string(max_error_quad), Point2f(10, 120));
					drawText(frame, "Visible: " + to_string(message_visible.data), Point2f(10, 140));
					drawText(frame, "Calibrated: " + to_string(calibrated), Point2f(10, 160));

					imshow("Aruco", frame);

					char key = (char) waitKey(1);

					if(key == 'q')
					{
						cosine_limit += 0.05;
					}
					else if(key == 'a')
					{
						cosine_limit -= 0.05;
					}

					if(key == 'w')
					{
						theshold_block_size += 2;
					}
					else if(key == 's' && theshold_block_size > 3)
					{
						theshold_block_size -= 2;
					}

					if(key == 'r')
					{
						max_error_quad += 0.005;
					}
					else if(key == 'f')
					{
						max_error_quad -= 0.005;
					}

					if(key == 'e')
					{
						min_area += 50;
					}
					else if(key == 'd')
					{
						min_area -= 50;
					}
				}
			}
			// ROS 2 差异：ROS_ERROR(...) -> RCLCPP_ERROR(logger, ...)
			catch(cv_bridge::Exception& e)
			{
				RCLCPP_ERROR(this->get_logger(), "Error getting image data");
			}
		}

		/**
		 * On camera info callback.
		 * Used to receive camera calibration parameters.
		 * ROS 2 差异：回调参数改为消息 SharedPtr（ROS 1 是 const sensor_msgs::CameraInfo&），
		 * 成员访问由 . 改为 ->。
		 */
		void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
		{
			if(!calibrated)
			{
				calibrated = true;

				for(unsigned int i = 0; i < 9; i++)
				{
					// ROS 2 差异：ROS 2 的 sensor_msgs/CameraInfo 字段名改为小写（K -> k）
					calibration.at<double>(i / 3, i % 3) = msg->k[i];
				}

				// ROS 2 差异：ROS 1 的 D 是定长 double[5]，ROS 2 的 d 是变长数组（长度取决于畸变
				// 模型），这里做长度检查后最多读取原来的 5 个参数，避免越界。
				unsigned int distortion_count = static_cast<unsigned int>(msg->d.size());
				if(distortion_count > 5)
				{
					distortion_count = 5;
				}

				for(unsigned int i = 0; i < distortion_count; i++)
				{
					distortion.at<double>(0, i) = msg->d[i];
				}

				if(debug)
				{
					cout << "Camera calibration param received" << endl;
					cout << "Camera: " << calibration << endl;
					cout << "Distortion: " << distortion << endl;
				}
			}
		}

		/**
		 * Callback to register markers on the marker list.
		 * This callback received a custom marker message.
		 * ROS 2 差异：aruco::Marker -> aruco::msg::Marker，回调参数为 SharedPtr。
		 */
		void onMarkerRegister(const aruco::msg::Marker::SharedPtr msg)
		{
			for(unsigned int i = 0; i < known.size(); i++)
			{
				if(known[i].id == msg->id)
				{
					known.erase(known.begin() + i);
					cout << "Marker " << to_string(msg->id) << " already exists, was replaced." << endl;
					break;
				}
			}

			known.push_back(ArucoMarkerInfo(msg->id, msg->size, Point3d(msg->posx, msg->posy, msg->posz), Point3d(msg->rotx, msg->roty, msg->rotz)));
			cout << "Marker " << to_string(msg->id) << " added." << endl;
		}

		/**
		 * Callback to remove markers from the marker list.
		 * Markers are removed by publishing the remove ID to the remove topic.
		 * ROS 2 差异：std_msgs::Int32 -> std_msgs::msg::Int32，回调参数为 SharedPtr。
		 */
		void onMarkerRemove(const std_msgs::msg::Int32::SharedPtr msg)
		{
			for(unsigned int i = 0; i < known.size(); i++)
			{
				if(known[i].id == msg->data)
				{
					known.erase(known.begin() + i);
					cout << "Marker " << to_string(msg->data) << " removed." << endl;
					break;
				}
			}
		}

		/**
		 * List of known of markers, to get the absolute position and rotation of the camera, some of these are required.
		 */
		vector<ArucoMarkerInfo> known = vector<ArucoMarkerInfo>();

		/**
		 * Camera calibration matrix.
		 */
		Mat calibration;

		/**
		 * Lenses distortion matrix.
		 */
		Mat distortion;

		/**
		 * Node visibility publisher.
		 * Publishes true when a known marker is visible, publishes false otherwise.
		 * ROS 2 差异：ros::Publisher -> rclcpp::Publisher<T>::SharedPtr，发布用 ->publish()。
		 */
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_visible;

		/**
		 * Node position publisher.
		 */
		rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_position;

		/**
		 * Node rotation publisher.
		 */
		rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_rotation;

		/**
		 * Node pose publisher.
		 */
		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;

		/**
		 * Node odometry publisher.
		 * Publishes the odometry of the tf_frame indicated using the pose calculated from marker.
		 */
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;

		/**
		 * Camera image subscriber.
		 * ROS 2 差异：原 image_transport::Subscriber -> rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr。
		 */
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera;

		/**
		 * Camera info subscriber.
		 */
		rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info;

		/**
		 * Marker register subscriber.
		 */
		rclcpp::Subscription<aruco::msg::Marker>::SharedPtr sub_marker_register;

		/**
		 * Marker remove subscriber.
		 */
		rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_marker_remove;

		/**
		 * Name of the transform tf name to indicate on published topics.
		 */
		string tf_frame_id;

		/**
		 * Pose publisher sequence counter.
		 * ROS 2 差异：std_msgs/Header 无 seq 字段，计数器保留但不再发布。
		 */
		int pub_pose_seq = 0;

		/**
		 * Flag to check if calibration parameters were received.
		 * If set to false the camera will be calibrated when a camera info message is received.
		 */
		bool calibrated = false;

		/**
		 * Flag to determine if OpenCV or ROS coordinates are used.
		 */
		bool use_opencv_coords = false;

		/**
		 * When debug parameter is se to true the node creates a new cv window to show debug information.
		 * By default is set to false.
		 * If set true the node will open a debug window.
		 */
		bool debug = false;

		/**
		 * Cosine limit used during the quad detection phase.
		 * Value between 0 and 1.
		 * By default 0.8 is used.
		 * The bigger the value more distortion tolerant the square detection will be.
		 */
		float cosine_limit = 0.7;

		/**
		 * Maximum error to be used by geometry poly aproximation method in the quad detection phase.
		 * By default 0.035 is used.
		 */
		float max_error_quad = 0.035;

		/**
		 * Adaptive theshold pre processing block size.
		 */
		int theshold_block_size = 0;

		/**
		 * Minimum threshold block size.
		 * By default 5 is used.
		 */
		int theshold_block_size_min = 3;

		/**
		 * Maximum threshold block size.
		 * By default 9 is used.
		 */
		int theshold_block_size_max = 21;

		/**
		 * Minimum area considered for aruco markers.
		 * Should be a value high enough to filter blobs out but detect the smallest marker necessary.
		 * By default 100 is used.
		 */
		int min_area = 100;
};

/**
 * Main method launches aruco ros node, the node gets image and calibration parameters from camera, and publishes position and rotation of the camera relative to the markers.
 * Units should be in meters and radians, the markers are described by a position and an euler rotation.
 * Position is also available as a pose message that should be easier to consume by other ROS nodes.
 * Its possible to pass markers as arugment to this node or register and remove them during runtime using another ROS node.
 * The coordinate system used by OpenCV uses Z+ to represent depth, Y- for height and X+ for lateral, but for the node the coordinate system used is diferent X+ for depth, Z+ for height and Y- for lateral movement.
 * The coordinates are converted on input and on output, its possible to force the OpenCV coordinate system by setting the use_opencv_coords param to true.
 *
 *           ROS          |          OpenCV
 *    Z+                  |    Y-
 *    |                   |    |
 *    |    X+             |    |    Z+
 *    |    /              |    |    /
 *    |   /               |    |   /
 *    |  /                |    |  /
 *    | /                 |    | /
 *    |/                  |    |/
 *    O-------------> Y-  |    O-------------> X+
 *
 * @param argc Number of arguments.
 * @param argv Value of the arguments.
 */
int main(int argc, char **argv)
{
	// ROS 2 差异：ros::init(argc, argv, "aruco") / ros::spin() -> rclcpp::init / rclcpp::spin(node)
	rclcpp::init(argc, argv);

	// ROS 2 差异：ROS 1 的全局参数服务器在 ROS 2 中没有对应物，参数是节点私有的。开启
	// automatically_declare_parameters_from_overrides 后，参数文件/命令行（launch）里给出的参数
	// 会被自动声明，从而让 has_parameter() 能够等价替代 ROS 1 的 nh.hasParam()
	// （用于可选的 calibration / distortion / markerN 参数）。
	rclcpp::NodeOptions options;
	options.automatically_declare_parameters_from_overrides(true);

	auto node = std::make_shared<ArucoNode>(options);

	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
