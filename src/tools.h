#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include <random>
#include <iostream>
#include <pcl/io/pcd_io.h>

#include "Eigen/Dense"
#include "render/render.h"

/// @brief Defines the measurement components for Lidar
struct lmarker
{
	double x, y; // Position X, Position Y
	lmarker(double setX, double setY)
		: x(setX), y(setY)
	{
	}
};

/// @brief Defines the measurement components for Radar
struct rmarker
{
	double rho, phi, rho_dot; // Radial Distance, Angle, Radial Velocity
	rmarker(double setRho, double setPhi, double setRhoDot)
		: rho(setRho), phi(setPhi), rho_dot(setRhoDot)
	{
	}
};

/// @brief Encapsulates implementations for sensing capabilities
class Tools
{
public:
	/// @brief Constructor for Tools
	Tools();

	/// @brief Destructor for Tools
	virtual ~Tools();

	// Members
	std::vector<Eigen::VectorXd> estimations;
	std::vector<Eigen::VectorXd> ground_truth;

	double noise(double stddev, long long seedNum);
	lmarker lidarSense(Car &car, pcl::visualization::PCLVisualizer::Ptr &viewer, long long timestamp, bool visualize);
	rmarker radarSense(Car &car, Car ego, pcl::visualization::PCLVisualizer::Ptr &viewer, long long timestamp, bool visualize);
	void ukfResults(Car car, pcl::visualization::PCLVisualizer::Ptr &viewer, double time, int steps);
	/**
	 * A helper method to calculate RMSE.
	 */
	Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);
	void savePcd(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string file);
	pcl::PointCloud<pcl::PointXYZ>::Ptr loadPcd(std::string file);
};

#endif /* TOOLS_H_ */
