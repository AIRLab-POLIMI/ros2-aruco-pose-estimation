// ROS2 imports
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// C++ imports
#include <Eigen/Dense>
#include <random>

// custom message definition
#include <aruco_interfaces/msg/aruco_markers.hpp>

class MultiArucoPlaneDetection : public rclcpp::Node {

private:
	// publisher of corrected aruco markers with optimal orientation perpendicular to the best fitting plane
	rclcpp::Publisher<aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_publisher_;

	// publisher to /viz_markers topic for display of computed plane
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher_;

	// subscription to /aruco_markers topic
	rclcpp::Subscription<aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscription_;

	const std::vector<int> required_markers_ids = {7, 8};					   // markers IDs required for plane detection
	const std::vector<int> markers_ids = {1, 2, 4, 5, 6, 7, 8};				   // markers IDs available
	const unsigned long expected_markers_max = markers_ids.size();			   // maximum number of markers expected in the scene
	const unsigned int expected_markers_min = required_markers_ids.size() + 2; // minimum number of required markers in the scene

	// RANSAC hyperparameters
	const int max_iterations = 25; // maximum number of iterations for RANSAC
	const double threshold = 0.05; // distance threshold in meters, perpendicular to the plane

	// name of the camera frame
	std::string camera_frame_name;

	const std::string viz_markers_topic = "/viz_markers"; // topic for visualization of the computed plane in RViz
	const std::string aruco_markers_topic = "/aruco/markers/small"; // topic for the detected aruco markers
	const std::string corrected_aruco_markers_topic = "/aruco/markers/corrected"; // topic for the corrected aruco markers

	// logger
	rclcpp::Logger LOGGER = rclcpp::get_logger("multi_aruco");

	const bool enable_visualization = false; // enable visualization of the computed plane in RViz

public:
	/**
	 * @brief constructor of the node
	 * @param node_options options for the node
	 */
	MultiArucoPlaneDetection(const rclcpp::NodeOptions &node_options);

	/**
	 * @brief callback function for /aruco/markers topic subscription.
	 * 		  Core thread of the node, it performs the plane detection and publishes the corrected aruco markers.
	 * @param msg message containing the detected aruco markers array
	 */
	void marker_callback(const aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

	/**
	 * @brief check if the detected markers are sufficient for plane detection
	 * @param msg message containing the detected aruco markers array
	 * @return true if the detected markers are sufficient, false otherwise
	 */
	bool areMarkersSufficient(const aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

	/**
	 * @brief construct a matrix of points from the detected aruco markers
	 * @param msg message containing the detected aruco markers array
	 * @return matrix of points, each column is a point [x,y,z]
	 */
	Eigen::MatrixXd constructOrderedMatrixFromPoints(const aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

	/**
	 * @brief compute the best fitting plane passing through all given points using RANSAC
	 * @param points matrix of points, each column is a point [x,y,z]
	 * @return the best fitting plane normal vector
	 */
	Eigen::Vector3d planeFittingWithRANSAC(const Eigen::MatrixXd &points);

	/**
	 * @brief compute the best fitting line passing through all given points using linear regression
	 * @param points matrix of points, each column is a point [x,y,z]
	 * @return the best fitting line normal vector
	 */
	Eigen::Vector3d lineFittingWithRegression(const Eigen::MatrixXd &points, const Eigen::Vector3d &plane_normal);

	/**
	 * @brief computex best fitting plane passing through all given points using SVD
	 * @param points matrix of points, each column is a normal vector
	 * @return the best fitting plane normal vector
	 */
	Eigen::Vector3d computeNormalWithSVD(const Eigen::MatrixXd &points);

	/**
	 * @brief compute the distance of a point [x,y,z] from a plane on its perpendicular direction
	 * @param normal normal vector of the plane
	 * @param point point to compute the distance from
	 * @return distance of the point from the plane
	 */
	double computeDistance(const Eigen::Vector3d &normal, const Eigen::Vector3d &point);

	/**
	 * @brief compute the centroid of a set of points
	 * @param points matrix of points, each column is a point [x,y,z]
	 * @return centroid of the points
	 */
	Eigen::Vector3d getCentroidFromPoints(const Eigen::MatrixXd &points);

	/**
	 * @brief set the yaw of a quaternion to zero, given a normal vector of a plane
	 * @param normal normal vector of the plane
	 * @return quaternion with yaw set to zero
	 */
	Eigen::Quaterniond setYawZero(const Eigen::Vector3d &normal);

	/**
	 * @brief construct a quaternion from two vectors and a normal vector
	 * @param vector_x first vector = best fitting line across arucos on the bottom of the plane
	 * @param vector_y second vector (must be perpendicular to the first one)
	 * @param normal_z normal vector to the best fitting plane
	 * @return quaternion with the given vectors as x and y axis and the normal vector as z axis
	 */
	Eigen::Quaterniond constructQuaternionFromVectors(const Eigen::Vector3d &vector_x,
													  const Eigen::Vector3d &vector_y,
													  const Eigen::Vector3d &normal_z);

	/**
	 * @brief visualize the quaternion orientation with a plane and an arrow in RViz using the /viz_markers topic
	 * @param quaternion quaternion to visualize as a plane with an arrow in RViz
	 * @param placement point where the center of the plane should be placed
	 */
	void visualizeQuaternionWithPlane(const Eigen::Quaterniond quaternion, const Eigen::Vector3d placement);

	/**
	 * @brief visualize the XYZ axes and a plane in RViz using the /viz_markers topic
	 * @param vx x axis vector
	 * @param vy y axis vector
	 * @param vz z axis vector = normal vector to the plane
	 * @param placement point where the center of the plane should be placed
	 */
	void visualizeAxesAndPlane(const Eigen::Vector3d vx, const Eigen::Vector3d vy, const Eigen::Vector3d vz,
							   const Eigen::Vector3d placement);
};