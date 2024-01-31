#include "multi_aruco_plane_detection.hpp"

MultiArucoPlaneDetection::MultiArucoPlaneDetection(const rclcpp::NodeOptions &node_options) : Node("multi_aruco_plane_detection", node_options) {

	// create a publisher for the corrected aruco markers
	aruco_publisher_ = this->create_publisher<ros2_aruco_interfaces::msg::ArucoMarkers>(
		"/aruco_markers_corrected", 10);

	// create a publisher for the visualization of the plane
	markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
		"/viz_markers", 10);

	// create a subscription to the aruco markers topic for their estimated pose and orientation
	aruco_subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
		"/aruco_markers", 10, std::bind(&MultiArucoPlaneDetection::marker_callback, this, std::placeholders::_1));

	camera_frame_name = this->get_parameter("camera_frame").as_string();
	if (camera_frame_name != std::string()) {
		RCLCPP_INFO(LOGGER, "Value of camera frame: %s", camera_frame_name.c_str());
	} else {
		RCLCPP_ERROR(LOGGER, "Failed to get camera frame parameter from config file");
	}
}

void MultiArucoPlaneDetection::marker_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
	// check whether the message contains at least 3 markers
	const int num_markers = msg->marker_ids.size();

	if (num_markers < 3 || num_markers > 7) {
		return; // not enough markers to compute a plane OR more markers than expected
	}

	// given the set of available markers, check whether the minimum subset of markers is available
	bool required_markers_available = true;
	for (unsigned int i = 0; i < required_markers_ids.size(); i++) {
		if (std::find(msg->marker_ids.begin(), msg->marker_ids.end(), required_markers_ids[i]) == msg->marker_ids.end()) {
			required_markers_available = false;
			break;
		}
	}
	if (!required_markers_available) {
		return; // insufficient markers to compute a plane with enough confidence
	}

	RCLCPP_INFO(this->get_logger(), "Received %d markers", num_markers);

	/*
	// compute the normal orientation of the plane with first method
	Eigen::Quaterniond q = computeNormalOrientation(p1, p2, p3);
	publishQuaternionAsArrow(q, placement);
	*/

	// create a matrix of points from the marker positions (x, y, z coordinates)
	Eigen::MatrixXd points(3, num_markers);
	for (int i = 0; i < num_markers; i++) {
		points(0, i) = msg->poses[i].position.x;
		points(1, i) = msg->poses[i].position.y;
		points(2, i) = msg->poses[i].position.z;
	}

	RCLCPP_DEBUG(this->get_logger(), "matrix points: %ld x %ld", points.rows(), points.cols());

	// compute the normal orientation of the plane
	// Eigen::Vector3d normal = computeNormalWithSVD(points);
	Eigen::Vector4d plane = planeFittingWithRANSAC(points);

	// convert eigen vector3d to eigen quaterniond
	//Eigen::Quaterniond q;
	//q.setFromTwoVectors(Eigen::Vector3d::UnitZ(), normal.normalized());

	/*
	// ----------------- visualize the normal to the plane in a predetermined position -----------------
	// create a eigen vector3d of points from the marker positions (x, y, z coordinates)
	const Eigen::Vector3d p1(msg->poses[0].position.x, msg->poses[0].position.y, msg->poses[0].position.z);
	const Eigen::Vector3d p2(msg->poses[1].position.x, msg->poses[1].position.y, msg->poses[1].position.z);
	const Eigen::Vector3d p3(msg->poses[2].position.x, msg->poses[2].position.y, msg->poses[2].position.z);

	// publish the quaternion as a visualization_msgs::Marker::ARROW
	geometry_msgs::msg::Point placement;
	placement.x = p1.x();
	placement.y = p1.y();
	placement.z = p1.z();
	this->publishQuaternionAsPlane(q, placement);
	*/

	this->visualizeVector4dWithPlaneAndNormal(plane);
}

// given a set of 3 points, computes the quaternion of the normal to the plane passing through these points
Eigen::Quaterniond MultiArucoPlaneDetection::computeNormalOrientation(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
																	  const Eigen::Vector3d &p3) {

	// Compute two vectors lying on the plane
	Eigen::Vector3d v1 = p2 - p1;
	Eigen::Vector3d v2 = p3 - p2;

	// Compute the normal vector to the plane
	Eigen::Vector3d normal = v1.cross(v2);

	// Compute the rotation quaternion from the z-axis to the normal vector
	Eigen::Quaterniond q;
	q.setFromTwoVectors(Eigen::Vector3d::UnitZ(), normal);
	return q;
}

// given a set of xyz points, computes all possible planes, then applies ransac and svd to find the best fitting plane
Eigen::Vector4d MultiArucoPlaneDetection::planeFittingWithRANSAC(const Eigen::MatrixXd &points) {
	const int n_points = points.cols();
	int n_combinations;
	if (n_points == 7) {
		n_combinations = 35;
	} else if (n_points == 6) {
		n_combinations = 20;
	} else if (n_points == 5) {
		n_combinations = 10;
	} else if (n_points == 4) {
		n_combinations = 4;
	} else { // points.size() == 3
		n_combinations = 1;
	}

	// given a set of points (x,y,z) compute the set of planes for every possible combination of 3 different points
	Eigen::MatrixXd planes(4, n_combinations);
	// Compute planes for every possible combination of 3 points
	
	int count = 0;
	for (long i = 0; i < n_points - 2; ++i) {
		for (long j = i + 1; j < n_points - 1; ++j) {
			for (long k = j + 1; k < n_points; ++k) {
				Eigen::Vector4d plane = findPlaneEquation(points.col(i), points.col(j), points.col(k));
				planes.col(count) = plane;
				count++;
			}
		}
	}

	RCLCPP_INFO(LOGGER, "planes matrix: %ld x %ld", planes.rows(), planes.cols());


	// RANSAC algorithm for plane fitting
	// https://en.wikipedia.org/wiki/Random_sample_consensus

	const int max_iterations = 100;
	const double threshold = 0.05; // distance threshold in meters, perpendicular to the plane

	// initialize the best model
	Eigen::Vector4d best_normal;
	int best_inliers = 0;
	double best_distance = 1000.0;
	int num_planes = planes.cols(); // = points.cols()
	int min_planes = std::ceil(num_planes / 2.0);

	// Random number generator beyween min_points and num_points
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> distribution(min_planes, num_planes);

	for (int i = 0; i < max_iterations; ++i) {
		// Randomly select a subset of points, with at least min_points and at most num_points
		int subset_size = distribution(gen);
		std::vector<int> subset_indices(num_planes); // initialize with all the markers
		for (int k = 0; k < num_planes; ++k) {
			subset_indices[k] = k;
		}
		std::shuffle(subset_indices.begin(), subset_indices.end(), gen); // select which markers to use

		// constructs subset of planes
		Eigen::MatrixXd subset(4, subset_size);
		for (int i_point = 0; i_point < subset_size; ++i_point) {
			int index = subset_indices[i_point];
			subset.col(i_point) = planes.col(index); // = points.col(index)
		}

		// Fit a plane to the subset
		//Eigen::Vector3d normal = computeNormalWithSVD(subset); // for points
		Eigen::Vector4d plane_normal = computePlaneNormalWithSVD(subset); // for planes
		//plane_normal = Eigen::Vector4d(plane_normal(0), plane_normal(1), plane_normal(2), plane_normal(3));

		RCLCPP_INFO(LOGGER, "plane normal: %f %f %f %f", plane_normal(0), plane_normal(1), plane_normal(2), plane_normal(3));

		// Count inliers and compute total distance metrics
		int inlier_count = 0;
		double total_distance = 0.0;
		for (int j = 0; j < n_points; ++j) {
			// using xyz points as elements of the matrix
			//double distance = computeDistance(normal, points.col(j)); // for points
			double distance = computeDistance(plane_normal, points.col(j));
			if (distance < threshold) {
				inlier_count++;
			}
			total_distance += distance;
		}

		// Update best plane with inlier count metric
		if (inlier_count > best_inliers) {
			best_inliers = inlier_count;
			best_normal = plane_normal;
		}

		if (total_distance < best_distance) {
			best_distance = total_distance;
			best_normal = plane_normal;
		}
	}

	return best_normal;
}

// dynamic sized double values matrix = set of points
// given a set of xyz points returns the normal to the best fitting plane
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

	return normalVector;
}


// dynamic sized double values matrix = set of points
// given a set of abcd planes returns the best fitting plane
Eigen::Vector4d MultiArucoPlaneDetection::computePlaneNormalWithSVD(const Eigen::MatrixXd &points) {
	// compute best fitting plane given a set of points
	// https://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points

	// Compute centroid
	Eigen::Vector4d centroid = points.rowwise().mean();
	// Subtract centroid
	Eigen::MatrixXd centeredPoints = points.colwise() - centroid;
	// Compute SVD
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(centeredPoints, Eigen::ComputeThinU | Eigen::ComputeThinV);
	// Extract the left singular vectors
	Eigen::MatrixXd leftSingularVectors = svd.matrixU();
	// Get the last column of the left singular vectors
	Eigen::Vector4d normalVector = leftSingularVectors.col(leftSingularVectors.cols() - 1);
	
	normalVector = normalVector + centroid;

	return normalVector;
}

// given a plane (vector4d) and a point (vector3d) computes the distance between the point and its perpendicular projection on the plane
double MultiArucoPlaneDetection::computeDistance(const Eigen::Vector4d &plane, const Eigen::Vector3d &point) {
	// Compute the distance from a point to a plane
	// https://mathinsight.org/distance_point_plane

	// Extract the coefficients of the plane equation
	double a = plane(0);
	double b = plane(1);
	double c = plane(2);
	double d = plane(3);

	// Compute the dot product of the point with the vector normal to the plane (a, b, c)
	double dotProduct = a * point.x() + b * point.y() + c * point.z();

	// Compute the distance using the formula
	double distance = std::abs(dotProduct + d) / std::sqrt(a * a + b * b + c * c);

	return distance;
}

// given a set of points (vector3d) computes the plane equation ax + by + cz + d = 0
Eigen::Vector4d MultiArucoPlaneDetection::findPlaneEquation(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2,
															const Eigen::Vector3d &point3) {
	// Define vectors from point1 to point2 and point3
	Eigen::Vector3d vec1 = point2 - point1;
	Eigen::Vector3d vec2 = point3 - point1;

	// Compute the normal vector to the plane
	Eigen::Vector3d normal = vec1.cross(vec2).normalized();

	// Compute the coefficients a, b, c of the plane equation ax + by + cz + d = 0
	double a = normal.x();
	double b = normal.y();
	double c = normal.z();

	// Compute d using one of the points
	double d = -(a * point1.x() + b * point1.y() + c * point1.z());

	return Eigen::Vector4d(a, b, c, d);
}

void MultiArucoPlaneDetection::visualizeQuaternionAsPlane(const Eigen::Quaterniond quaternion,
														geometry_msgs::msg::Point placement) {
	// create a visualization markery array, so to display a plane and the normal vector

	visualization_msgs::msg::Marker marker_plane;
	marker_plane.header.frame_id = camera_frame_name;
	marker_plane.header.stamp = this->now();
	marker_plane.id = 0;
	marker_plane.type = visualization_msgs::msg::Marker::CUBE;
	marker_plane.action = visualization_msgs::msg::Marker::ADD;
	marker_plane.pose.position = placement;
	marker_plane.pose.orientation.x = quaternion.x();
	marker_plane.pose.orientation.y = quaternion.y();
	marker_plane.pose.orientation.z = quaternion.z();
	marker_plane.pose.orientation.w = quaternion.w();
	marker_plane.scale.x = 0.5;
	marker_plane.scale.y = 0.5;
	marker_plane.scale.z = 0.01;
	// orange color rgb = (255, 165, 0)
	marker_plane.color.r = 255.0 / 255.0;
	marker_plane.color.g = 165.0 / 255.0;
	marker_plane.color.b = 0.0;
	marker_plane.color.a = 1.0;

	visualization_msgs::msg::Marker marker_arrow;
	marker_arrow.header.frame_id = camera_frame_name;
	marker_arrow.header.stamp = this->now();
	marker_arrow.id = 1;
	marker_arrow.type = visualization_msgs::msg::Marker::ARROW;
	marker_arrow.action = visualization_msgs::msg::Marker::ADD;
	marker_arrow.pose.position = placement;
	marker_arrow.pose.orientation.x = quaternion.x();
	marker_arrow.pose.orientation.y = quaternion.y();
	marker_arrow.pose.orientation.z = quaternion.z();
	marker_arrow.pose.orientation.w = quaternion.w();
	marker_arrow.scale.x = 0.6;
	marker_arrow.scale.y = 0.01;
	marker_arrow.scale.z = 0.01;
	// color
	marker_arrow.color.r = 0.0;
	marker_arrow.color.g = 255.0 / 255.0;
	marker_arrow.color.b = 100.0 / 255.0;
	marker_arrow.color.a = 1.0;

	visualization_msgs::msg::MarkerArray marker_array_msg;
	marker_array_msg.markers.resize(2);
	marker_array_msg.markers[0] = marker_plane;
	marker_array_msg.markers[1] = marker_arrow;

	// publish the message
	markers_publisher_->publish(marker_array_msg);
}

void MultiArucoPlaneDetection::visualizeVector4dWithPlaneAndNormal(Eigen::Vector4d plane) {

	Eigen::Vector3d normal(plane(0), plane(1), plane(2));
	normal.normalize();
	Eigen::Quaterniond normal_quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), normal);
	Eigen::Vector3d closest_point(-normal.x() * plane(3), -normal.y() * plane(3), -normal.z() * plane(3));

	visualization_msgs::msg::Marker marker_plane;
	marker_plane.header.frame_id = camera_frame_name;
	marker_plane.header.stamp = this->now();
	marker_plane.id = 0;
	marker_plane.type = visualization_msgs::msg::Marker::CUBE;
	marker_plane.action = visualization_msgs::msg::Marker::ADD;
	marker_plane.pose.position.x = closest_point.x();
	marker_plane.pose.position.y = closest_point.y();
	marker_plane.pose.position.z = closest_point.z();
	marker_plane.pose.orientation.x = normal_quat.x();
	marker_plane.pose.orientation.y = normal_quat.y();
	marker_plane.pose.orientation.z = normal_quat.z();
	marker_plane.pose.orientation.w = normal_quat.w();
	marker_plane.scale.x = 0.5;
	marker_plane.scale.y = 0.5;
	marker_plane.scale.z = 0.01;
	// orange color rgb = (255, 165, 0)
	marker_plane.color.r = 255.0 / 255.0;
	marker_plane.color.g = 165.0 / 255.0;
	marker_plane.color.b = 0.0;
	marker_plane.color.a = 1.0;

	visualization_msgs::msg::Marker marker_arrow;
	marker_arrow.header.frame_id = camera_frame_name;
	marker_arrow.header.stamp = this->now();
	marker_arrow.id = 1;
	marker_arrow.type = visualization_msgs::msg::Marker::ARROW;
	marker_arrow.action = visualization_msgs::msg::Marker::ADD;
	marker_arrow.pose.position.x = closest_point.x();
	marker_arrow.pose.position.y = closest_point.y();
	marker_arrow.pose.position.z = closest_point.z();
	marker_arrow.pose.orientation.x = normal_quat.x();
	marker_arrow.pose.orientation.y = normal_quat.y();
	marker_arrow.pose.orientation.z = normal_quat.z();
	marker_arrow.pose.orientation.w = normal_quat.w();
	marker_arrow.scale.x = 0.6;
	marker_arrow.scale.y = 0.01;
	marker_arrow.scale.z = 0.01;
	// color
	marker_arrow.color.r = 0.0;
	marker_arrow.color.g = 255.0 / 255.0;
	marker_arrow.color.b = 100.0 / 255.0;
	marker_arrow.color.a = 1.0;

	visualization_msgs::msg::MarkerArray marker_array_msg;
	marker_array_msg.markers.resize(2);
	marker_array_msg.markers[0] = marker_plane;
	marker_array_msg.markers[1] = marker_arrow;

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