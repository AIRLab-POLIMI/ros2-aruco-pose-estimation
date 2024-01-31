// ROS2 imports
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// C++ imports
#include <Eigen/Dense>
#include <random>

// custom message definition
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>

class MultiArucoPlaneDetection : public rclcpp::Node {

private:
	// publisher of corrected aruco markers with optimal orientation perpendicular to the best fitting plane
	rclcpp::Publisher<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_publisher_;

	// publisher to /viz_markers topic for display of computed plane
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher_;

	// subscription to /aruco_markers topic
	rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscription_;

	const std::vector<int> required_markers_ids = {7, 8};
	const std::vector<int> markers_used = {1, 2, 4, 5, 6, 7, 8};

	// name of the camera frame
	std::string camera_frame_name;

	// logger
	rclcpp::Logger LOGGER = rclcpp::get_logger("multi_aruco");

public:
	// callback function for /aruco_markers topic
	void marker_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

	// compute plane passing through 3 given points
	// given a set of 3 points, computes the quaternion of the normal to the plane passing through these points
	Eigen::Vector3d computeNormalFromPlanePoints(const Eigen::Vector3d &p1,
												 const Eigen::Vector3d &p2,
												 const Eigen::Vector3d &p3);

	Eigen::MatrixXd getVectors3dFromPoints(const Eigen::MatrixXd &points);

	Eigen::Vector3d planeFittingWithRANSAC(const Eigen::MatrixXd &points);

	// compute best fitting plane passing through all given points using SVD
	Eigen::Vector3d computeNormalWithSVD(const Eigen::MatrixXd &points);

	Eigen::Vector4d computePlaneNormalWithSVD(const Eigen::MatrixXd &points);

	Eigen::Vector3d getCentroidFromPoints(const Eigen::MatrixXd &points);

	double computeDistance(const Eigen::Vector4d &plane, const Eigen::Vector3d &point);

	double computeDistance(const Eigen::Vector3d &normal, const Eigen::Vector3d &point);

	void visualizeVector3dWithPlane(const Eigen::Vector3d normal, geometry_msgs::msg::Point placement);

	void visualizeVector4dWithPlaneAndNormal(Eigen::Vector4d plane);

	Eigen::Vector4d findPlaneEquation(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2,
									  const Eigen::Vector3d &point3);

	MultiArucoPlaneDetection(const rclcpp::NodeOptions &node_options);
};