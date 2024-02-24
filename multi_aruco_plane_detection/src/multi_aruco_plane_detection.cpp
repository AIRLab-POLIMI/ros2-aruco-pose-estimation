#include "multi_aruco_plane_detection.hpp"

/**
 * @brief constructor of the node
 * @param node_options options for the node
 */
MultiArucoPlaneDetection::MultiArucoPlaneDetection(const rclcpp::NodeOptions &node_options)
	: Node("multi_aruco_plane_detection", node_options) {

	// create a publisher for the corrected aruco markers
	aruco_publisher_ = this->create_publisher<aruco_interfaces::msg::ArucoMarkers>(
		this->corrected_aruco_markers_topic, 10);

	// create a publisher for the visualization of the plane
	markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
		this->viz_markers_topic, 10);

	// create a subscription to the aruco markers topic for their estimated pose and orientation
	aruco_subscription_ = this->create_subscription<aruco_interfaces::msg::ArucoMarkers>(
		this->aruco_markers_topic, 10, std::bind(&MultiArucoPlaneDetection::marker_callback, this, std::placeholders::_1));

	// get the name of the camera frame from the launch parameter
	camera_frame_name = this->get_parameter("camera_frame").as_string();
	if (camera_frame_name != std::string()) {
		RCLCPP_INFO(LOGGER, "Value of camera frame: %s", camera_frame_name.c_str());
	} else {
		RCLCPP_ERROR(LOGGER, "Failed to get camera frame parameter from config file");
	}
}

/**
 * @brief callback function for /aruco/markers topic subscription.
 * 		  Core thread of the node, it performs the plane detection and publishes the corrected aruco markers.
 * @param msg message containing the detected aruco markers array
 */
void MultiArucoPlaneDetection::marker_callback(const aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
	// check if the detected markers are sufficient for plane detection
	bool valid_markers = areMarkersSufficient(msg);

	// skip the computation if the markers are not sufficient
	if (!valid_markers) {
		return;
	} else {
		RCLCPP_DEBUG(LOGGER, "Received %ld markers", msg->marker_ids.size());
	}

	// create a matrix of points from the markers, ordered according to the saved markers IDs
	Eigen::MatrixXd points = constructOrderedMatrixFromPoints(msg);
	// compute the normal orientation of the best fitting plane to the given points
	Eigen::Vector3d normal3d = planeFittingWithRANSAC(points);
	// compute the vector x of the plane given the aruco positions and the normal vector to the plane
	Eigen::Vector3d vector_x = lineFittingWithRegression(points, normal3d);
	// compute the vector y as the cross product between the normal and the vector x
	Eigen::Vector3d vector_y = normal3d.cross(vector_x);
	// get the quaternion from the combination of the 3 vectors
	Eigen::Quaterniond normal_quat = constructQuaternionFromVectors(vector_x, vector_y, normal3d);

	// visualize the plane with the normal vector in the choses position (first marker registered)
	if (enable_visualization) {
		this->visualizeQuaternionWithPlane(normal_quat, getCentroidFromPoints(points));

		// for debugging: visualize the axes and the plane composing the final orientation computed
		// this->visualizeAxesAndPlane(vector_x, vector_y, normal3d, getCentroidFromPoints(points));
	}

	// create a corrected aruco marker message and publish it with the corrected orientation
	aruco_interfaces::msg::ArucoMarkers corrected_markers(*msg);
	for (unsigned int i = 0; i < msg->marker_ids.size(); i++) {
		// convert eigen quaterniond to geometry_msgs quaternion
		geometry_msgs::msg::Quaternion quaternion_msg;
		quaternion_msg.x = normal_quat.x();
		quaternion_msg.y = normal_quat.y();
		quaternion_msg.z = normal_quat.z();
		quaternion_msg.w = normal_quat.w();

		// create a corrected aruco marker message by changing only the orientation
		corrected_markers.poses[i].orientation = quaternion_msg;
	}
	// publish corrected markers
	aruco_publisher_->publish(corrected_markers);
}

/**
 * @brief check if the detected markers are sufficient for plane detection
 * @param msg message containing the detected aruco markers array
 * @return true if the detected markers are sufficient, false otherwise
 */
bool MultiArucoPlaneDetection::areMarkersSufficient(const aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
	// check whether the message contains at least 3 markers
	const unsigned int num_markers = msg->marker_ids.size();

	// at least 3 markers are needed to compute a valid plane
	// at most 7 markers are expected with the current setup
	if (num_markers < expected_markers_min || num_markers > expected_markers_max) {
		RCLCPP_DEBUG(LOGGER, "Received %d markers: insufficient or invalid quantity", num_markers);
		return false; // not enough markers to compute a plane OR more markers than expected
	}

	//  given the set of available markers, check whether the minimum subset of markers is available
	bool plane_markers_available = true;
	for (unsigned int i = 0; i < required_markers_ids.size(); i++) {
		if (std::find(msg->marker_ids.begin(), msg->marker_ids.end(), required_markers_ids[i]) == msg->marker_ids.end()) {
			plane_markers_available = false;
			break;
		}
	}
	if (!plane_markers_available) {
		return false; // insufficient markers to compute a plane with enough confidence
	}

	// check if at least 2 markers are available to compute the line, among the marker ids not in the required set
	int line_markers_available = 0;
	for (unsigned int i = 0; i < markers_ids.size() - 2; i++) {
		for (unsigned int j = 0; j < num_markers; j++) {
			if (markers_ids[i] == msg->marker_ids[j]) {
				line_markers_available += 1;
				break;
			}
		}
	}
	if (line_markers_available < 2) {
		return false; // insufficient markers to compute a line with enough confidence
	}

	return true; // sufficient markers to compute a plane and a line
}

/**
 * @brief construct a matrix of points from the detected aruco markers
 * @param msg message containing the detected aruco markers array
 * @return matrix of points, each column is a point [x,y,z]
 */
Eigen::MatrixXd MultiArucoPlaneDetection::constructOrderedMatrixFromPoints(const aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
	// create a matrix of points from the markers
	// order the points in the matrix according to the order of the markers IDs
	const unsigned int num_markers = msg->marker_ids.size();
	Eigen::MatrixXd points(3, num_markers);
	for (unsigned int i = 0, position = 0; i < markers_ids.size(); i++) {
		// get the index of the marker ID in the array of available markers
		int marker_index = -1;
		for (unsigned int j = 0; j < num_markers; j++) {
			if (markers_ids[i] == msg->marker_ids[j]) {
				marker_index = j;
				break;
			}
		}
		if (marker_index != -1) {
			// fill the matrix of points with the xyz coordinates of the markers
			points.col(position) << msg->poses[marker_index].position.x,
				msg->poses[marker_index].position.y, msg->poses[marker_index].position.z;
			position++;
		} else {
			// this marker is not available, so skip it
		}
	}
	return points;
}

/**
 * @brief compute the best fitting plane passing through all given points using RANSAC
 * @param points matrix of points, each column is a point [x,y,z]
 * @return the best fitting plane normal vector
 */
Eigen::Vector3d MultiArucoPlaneDetection::planeFittingWithRANSAC(const Eigen::MatrixXd &points) {

	// RANSAC algorithm for plane fitting
	// https://en.wikipedia.org/wiki/Random_sample_consensus

	// initialize the worst base model for RANSAC
	Eigen::Vector3d best_normal;
	int best_inliers = 0;
	double best_distance = 1000.0;

	// range of quantity of normal vectors to use for RANSAC
	const int num_points = points.cols(); // = points.cols()
	const int min_points = std::ceil(num_points / 2.0);

	const Eigen::Vector3d centroid = getCentroidFromPoints(points);

	// Random number generator beyween min_points and num_points
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> distribution(min_points, num_points);

	// create vector of indexes to select a random subset of points
	std::vector<int> subset_indices(num_points); // initialize with all the markers
	for (int k = 0; k < num_points; ++k) {
		subset_indices[k] = k;
	}

	for (int i = 0; i < max_iterations; ++i) {
		// Randomly select a subset of points, with at least min_points and at most num_points
		int subset_size = distribution(gen);
		std::shuffle(subset_indices.begin(), subset_indices.end(), gen); // select which markers to use

		// constructs subset of planes
		Eigen::MatrixXd subset(3, subset_size);
		for (int i_point = 0; i_point < subset_size; ++i_point) {
			int index = subset_indices[i_point];
			subset.col(i_point) = points.col(index);
		}

		// Fit a plane to the subset
		Eigen::Vector3d normal3d = computeNormalWithSVD(subset); // for points

		// Count inliers and compute total distance metrics
		int inlier_count = 0;
		double total_distance = 0.0;
		for (int j = 0; j < num_points; ++j) { // for every registered point
			// using xyz points as elements of the matrix
			// compute distance between plane (normal) and point
			Eigen::Vector3d point_normalized = points.col(j) - centroid;
			double distance = computeDistance(normal3d, point_normalized);
			if (distance < threshold) {
				inlier_count++;
			}
			total_distance += distance;
		}

		//  Update best plane with inlier count metric
		if (inlier_count > best_inliers) {
			best_inliers = inlier_count;
			best_normal = normal3d;
		}

		if (total_distance < best_distance) {
			best_distance = total_distance;
			// best_normal = normal3d;
		}
	}

	return best_normal;
}

/**
 * @brief computex best fitting plane passing through all given points using SVD
 * @param points matrix of points, each column is a normal vector
 * @return the best fitting plane normal vector
 */
Eigen::Vector3d MultiArucoPlaneDetection::computeNormalWithSVD(const Eigen::MatrixXd &points) {
	// compute best fitting plane given a set of points
	// https://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points

	// Compute centroid
	Eigen::Vector3d centroid = points.rowwise().mean();
	// Subtract centroid
	Eigen::MatrixXd centeredPoints = points.colwise() - centroid;
	// Compute SVD
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(centeredPoints, Eigen::ComputeThinU | Eigen::ComputeThinV);
	// Extract the left singular vectors
	Eigen::MatrixXd leftSingularVectors = svd.matrixU();
	// Get the last column of the left singular vectors
	Eigen::Vector3d normalVector = leftSingularVectors.col(leftSingularVectors.cols() - 1);

	// set direction of the normal vector (negative with respect to the frame)
	if (normalVector.z() > 0) {
		normalVector = -normalVector;
	}

	return normalVector;
}

/**
 * @brief compute the best fitting line passing through all given points using linear regression
 * @param points matrix of points, each column is a point [x,y,z]
 * @return the best fitting line normal vector
 */
Eigen::Vector3d MultiArucoPlaneDetection::lineFittingWithRegression(const Eigen::MatrixXd &points,
																	const Eigen::Vector3d &plane_normal) {

	// remove the last two columns of the matrix: it will produce the matrix of the points on the bottom
	// 		of the aruco button setup, excluding the	two points on top of the aruco button setup
	const int num_valid_markers = points.cols() - 2;

	// Compute the centroid of the points
	Eigen::Vector3d centroid(0.0, 0.0, 0.0);
	for (int i = 0; i < num_valid_markers; i++) {
		centroid += points.col(i);
	}
	centroid /= num_valid_markers;

	// normalize the plane normal
	double plane_norm = plane_normal.norm();
	double plane_norm_squared = plane_norm * plane_norm;
	Eigen::Vector3d normal = plane_normal / plane_norm;

	// then project the points on the plane normal and get the matrix of projected points coordinates
	Eigen::MatrixXd projected_points(3, num_valid_markers);
	for (int i = 0; i < num_valid_markers; i++) {
		// remove the centroid from the points
		Eigen::Vector3d centered_point = points.col(i) - centroid;
		// orthogonal projection of the point on the plane
		projected_points.col(i) = centered_point - centered_point.dot(normal) * normal / (plane_norm_squared);
	}

	// compute centroid of projected points
	centroid = Eigen::Vector3d::Zero();
	for (int i = 0; i < num_valid_markers; i++) {
		centroid += projected_points.col(i);
	}
	centroid /= num_valid_markers;

	// Compute the covariance matrix of the projected points
	Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
	for (int i = 0; i < num_valid_markers; i++) {
		Eigen::Vector3d diff = projected_points.col(i) - centroid;
		covariance += diff * diff.transpose();
	}
	covariance /= (num_valid_markers - 1);

	// Compute the eigenvectors of the covariance matrix
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
	// Take the last eigenvector as the direction of greatest variance = the best fitting line
	Eigen::Vector3d vector_x = solver.eigenvectors().col(2);

	// use the last 2 marker points coordinates to compute the direction of the vector x
	Eigen::Vector3d point_a = points.col(num_valid_markers);
	Eigen::Vector3d point_b = points.col(num_valid_markers + 1);

	// compute the direction of the vector x
	if (point_a.x() > point_b.x()) {
		vector_x = -vector_x;
	}

	return vector_x;
}

/**
 * @brief compute the distance of a point [x,y,z] from a plane on its perpendicular direction
 * @param normal normal vector of the plane
 * @param point point to compute the distance from
 * @return distance of the point from the plane
 */
double MultiArucoPlaneDetection::computeDistance(const Eigen::Vector3d &normal, const Eigen::Vector3d &point) {
	// Compute the distance from a point to a plane
	// Assuming the point is part of a constellation of points where the centroid is the origin
	// https://mathinsight.org/distance_point_plane

	// Compute the dot product of the point with the vector normal to the plane (a, b, c)
	double dotProduct = normal.x() * point.x() + normal.y() * point.y() + normal.z() * point.z();
	double norm = std::sqrt(normal.x() * normal.x() + normal.y() * normal.y() + normal.z() * normal.z());

	// Compute the distance using the formula
	double distance = std::abs(dotProduct) / norm;

	return distance;
}

/**
 * @brief compute the centroid of a set of points
 * @param points matrix of points, each column is a point [x,y,z]
 * @return centroid of the points
 */
Eigen::Vector3d MultiArucoPlaneDetection::getCentroidFromPoints(const Eigen::MatrixXd &points) {
	// Compute centroid
	Eigen::Vector3d centroid = points.rowwise().mean();
	return centroid;
}

/**
 * @brief construct a quaternion from two vectors and a normal vector
 * @param vector_x first vector = best fitting line across arucos on the bottom of the plane
 * @param vector_y second vector (must be perpendicular to the first one)
 * @param normal_z normal vector to the best fitting plane
 * @return quaternion with the given vectors as x and y axis and the normal vector as z axis
 */
Eigen::Quaterniond MultiArucoPlaneDetection::constructQuaternionFromVectors(const Eigen::Vector3d &vector_x,
																			const Eigen::Vector3d &vector_y,
																			const Eigen::Vector3d &normal_z) {
	// given the 3 vectors, construct a quaternion with the given vectors as x and y axis and the normal vector as z axis
	Eigen::Matrix3d rotation_matrix;
	rotation_matrix.col(0) = vector_x.normalized();
	rotation_matrix.col(1) = vector_y.normalized();
	rotation_matrix.col(2) = normal_z.normalized();
	Eigen::Quaterniond quaternion(rotation_matrix);
	return quaternion;
}

/**
 * @brief visualize the quaternion orientation with a plane and an arrow in RViz using the /viz_markers topic
 * @param quaternion quaternion to visualize as a plane with an arrow in RViz
 * @param placement point where the center of the plane should be placed
 */
void MultiArucoPlaneDetection::visualizeQuaternionWithPlane(const Eigen::Quaterniond quaternion,
															const Eigen::Vector3d placement) {
	// create a visualization markery array, so to display a plane and the normal vector
	visualization_msgs::msg::Marker marker_plane;
	marker_plane.header.frame_id = camera_frame_name;
	marker_plane.header.stamp = this->now();
	marker_plane.id = 0;
	marker_plane.type = visualization_msgs::msg::Marker::CUBE;
	marker_plane.action = visualization_msgs::msg::Marker::ADD;
	marker_plane.pose.position.x = placement.x();
	marker_plane.pose.position.y = placement.y();
	marker_plane.pose.position.z = placement.z();
	marker_plane.pose.orientation.x = quaternion.x();
	marker_plane.pose.orientation.y = quaternion.y();
	marker_plane.pose.orientation.z = quaternion.z();
	marker_plane.pose.orientation.w = quaternion.w();
	marker_plane.scale.x = 0.5;
	marker_plane.scale.y = 0.5;
	marker_plane.scale.z = 0.01;
	// orange color rgb = (255, 165, 0)
	marker_plane.color.r = 56.0 / 255.0;
	marker_plane.color.g = 125.0 / 255.0;
	marker_plane.color.b = 187.0 / 255.0;
	marker_plane.color.a = 1.0;

	visualization_msgs::msg::Marker marker_arrow;
	marker_arrow.header.frame_id = camera_frame_name;
	marker_arrow.header.stamp = this->now();
	marker_arrow.id = 1;
	marker_arrow.type = visualization_msgs::msg::Marker::ARROW;
	marker_arrow.action = visualization_msgs::msg::Marker::ADD;
	marker_arrow.pose.position.x = placement.x();
	marker_arrow.pose.position.y = placement.y();
	marker_arrow.pose.position.z = placement.z();
	marker_arrow.pose.orientation.x = quaternion.x();
	marker_arrow.pose.orientation.y = quaternion.y();
	marker_arrow.pose.orientation.z = quaternion.z();
	marker_arrow.pose.orientation.w = quaternion.w();
	marker_arrow.scale.x = 0.6;
	marker_arrow.scale.y = 0.01;
	marker_arrow.scale.z = 0.01;
	// color greenish
	marker_arrow.color.r = 0.0;
	marker_arrow.color.g = 255.0 / 255.0;
	marker_arrow.color.b = 100.0 / 255.0;
	marker_arrow.color.a = 1.0;

	// create a marker array message
	visualization_msgs::msg::MarkerArray marker_array_msg;
	marker_array_msg.markers.resize(2);
	marker_array_msg.markers[0] = marker_plane;
	marker_array_msg.markers[1] = marker_arrow;

	// publish the message
	markers_publisher_->publish(marker_array_msg);
}

/**
 * @brief visualize the XYZ axes and a plane in RViz using the /viz_markers topic
 * @param vx x axis vector
 * @param vy y axis vector
 * @param vz z axis vector = normal vector to the plane
 * @param placement point where the center of the plane should be placed
 */
void MultiArucoPlaneDetection::visualizeAxesAndPlane(const Eigen::Vector3d vx, const Eigen::Vector3d vy,
													 const Eigen::Vector3d vz, const Eigen::Vector3d placement) {

	// convert each eigen vector3d to eigen quaterniond
	Eigen::Quaterniond quaternion_z;
	quaternion_z.setFromTwoVectors(Eigen::Vector3d::UnitZ(), vz.normalized());

	// create a visualization markery array, so to display a plane and the normal vector
	visualization_msgs::msg::Marker marker_plane;
	marker_plane.header.frame_id = camera_frame_name;
	marker_plane.header.stamp = rclcpp::Clock().now();
	marker_plane.id = 0;
	marker_plane.type = visualization_msgs::msg::Marker::CUBE;
	marker_plane.action = visualization_msgs::msg::Marker::ADD;
	marker_plane.pose.position.x = placement.x();
	marker_plane.pose.position.y = placement.y();
	marker_plane.pose.position.z = placement.z();
	marker_plane.pose.orientation.x = quaternion_z.x();
	marker_plane.pose.orientation.y = quaternion_z.y();
	marker_plane.pose.orientation.z = quaternion_z.z();
	marker_plane.pose.orientation.w = quaternion_z.w();
	marker_plane.scale.x = 0.5;
	marker_plane.scale.y = 0.5;
	marker_plane.scale.z = 0.01;
	// orange color rgb = (255, 165, 0)
	marker_plane.color.r = 255.0 / 255.0;
	marker_plane.color.g = 165.0 / 255.0;
	marker_plane.color.b = 0.0;
	marker_plane.color.a = 1.0;

	// rotate the y axis arrow such that it represents the green y axis
	// quaternion_x = quaternion_x * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());

	Eigen::Quaterniond quaternion_x;
	quaternion_x.setFromTwoVectors(Eigen::Vector3d::UnitX(), vx.normalized());

	visualization_msgs::msg::Marker marker_arrow_x;
	marker_arrow_x.header.frame_id = camera_frame_name;
	marker_arrow_x.header.stamp = rclcpp::Clock().now();
	marker_arrow_x.id = 1;
	marker_arrow_x.type = visualization_msgs::msg::Marker::ARROW;
	marker_arrow_x.action = visualization_msgs::msg::Marker::ADD;
	marker_arrow_x.pose.position.x = placement.x();
	marker_arrow_x.pose.position.y = placement.y();
	marker_arrow_x.pose.position.z = placement.z();
	marker_arrow_x.pose.orientation.x = quaternion_x.x();
	marker_arrow_x.pose.orientation.y = quaternion_x.y();
	marker_arrow_x.pose.orientation.z = quaternion_x.z();
	marker_arrow_x.pose.orientation.w = quaternion_x.w();
	marker_arrow_x.scale.x = 0.6;
	marker_arrow_x.scale.y = 0.01;
	marker_arrow_x.scale.z = 0.01;
	// color red
	marker_arrow_x.color.r = 255.0 / 255.0;
	marker_arrow_x.color.g = 0.0 / 255.0;
	marker_arrow_x.color.b = 0.0 / 255.0;
	marker_arrow_x.color.a = 1.0;

	Eigen::Quaterniond quaternion_y;
	quaternion_y.setFromTwoVectors(Eigen::Vector3d::UnitX(), vy.normalized());

	visualization_msgs::msg::Marker marker_arrow_y;
	marker_arrow_y.header.frame_id = camera_frame_name;
	marker_arrow_y.header.stamp = rclcpp::Clock().now();
	marker_arrow_y.id = 2;
	marker_arrow_y.type = visualization_msgs::msg::Marker::ARROW;
	marker_arrow_y.action = visualization_msgs::msg::Marker::ADD;
	marker_arrow_y.pose.position.x = placement.x();
	marker_arrow_y.pose.position.y = placement.y();
	marker_arrow_y.pose.position.z = placement.z();
	marker_arrow_y.pose.orientation.x = quaternion_y.x();
	marker_arrow_y.pose.orientation.y = quaternion_y.y();
	marker_arrow_y.pose.orientation.z = quaternion_y.z();
	marker_arrow_y.pose.orientation.w = quaternion_y.w();
	marker_arrow_y.scale.x = 0.6;
	marker_arrow_y.scale.y = 0.01;
	marker_arrow_y.scale.z = 0.01;
	// color green
	marker_arrow_y.color.r = 0.0 / 255.0;
	marker_arrow_y.color.g = 255.0 / 255.0;
	marker_arrow_y.color.b = 0.0 / 255.0;
	marker_arrow_y.color.a = 1.0;

	quaternion_z.setFromTwoVectors(Eigen::Vector3d::UnitX(), vz.normalized());

	visualization_msgs::msg::Marker marker_arrow_z;
	marker_arrow_z.header.frame_id = camera_frame_name;
	marker_arrow_z.header.stamp = rclcpp::Clock().now();
	marker_arrow_z.id = 3;
	marker_arrow_z.type = visualization_msgs::msg::Marker::ARROW;
	marker_arrow_z.action = visualization_msgs::msg::Marker::ADD;
	marker_arrow_z.pose.position.x = placement.x();
	marker_arrow_z.pose.position.y = placement.y();
	marker_arrow_z.pose.position.z = placement.z();
	marker_arrow_z.pose.orientation.x = quaternion_z.x();
	marker_arrow_z.pose.orientation.y = quaternion_z.y();
	marker_arrow_z.pose.orientation.z = quaternion_z.z();
	marker_arrow_z.pose.orientation.w = quaternion_z.w();
	marker_arrow_z.scale.x = 0.6;
	marker_arrow_z.scale.y = 0.01;
	marker_arrow_z.scale.z = 0.01;
	// color blue
	marker_arrow_z.color.r = 0.0;
	marker_arrow_z.color.g = 0.0;
	marker_arrow_z.color.b = 255.0 / 255.0;
	marker_arrow_z.color.a = 1.0;

	// create a marker array message
	visualization_msgs::msg::MarkerArray marker_array_msg;
	marker_array_msg.markers.resize(4);
	marker_array_msg.markers[0] = marker_plane;
	marker_array_msg.markers[1] = marker_arrow_x;
	marker_array_msg.markers[2] = marker_arrow_y;
	marker_array_msg.markers[3] = marker_arrow_z;

	// publish the message
	markers_publisher_->publish(marker_array_msg);
}

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	// read parameters
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);

	// Create an instance of the node
	auto node = std::make_shared<MultiArucoPlaneDetection>(node_options);

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}