#include <algorithm>

#include "highway.h"
#include "matplotlibcpp.h"

// Use shorter alias for matplotlib-cpp
namespace plt = matplotlibcpp;

int main(int argc, char **argv)
{
	// Initialize PCLVisualizer for visualization
	pcl::visualization::PCLVisualizer::Ptr viewer = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
	viewer->setBackgroundColor(0, 0, 0);

	// Set camera position and angle for the visualizer
	viewer->initCameraParameters();
	float x_pos = 0;
	viewer->setCameraPosition(x_pos - 26, 0, 15.0, x_pos + 25, 0, 0, 0, 0, 1);

	// Initialize highway with the visualizer
	Highway highway(viewer);

	// Define properties for visualization loop
	int frame_per_sec = 30;
	int sec_interval = 10;
	int frame_count = 0;
	int time_us = 0;
	double egoVelocity = 25;

	// NIS Metric preparation begin
	highway.InitNIS(frame_per_sec * sec_interval);
	// NIS Metric preparation end

	// Define visualization loop
	while (frame_count < (frame_per_sec * sec_interval))
	{
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		highway.stepHighway(egoVelocity, time_us, frame_per_sec, viewer);
		viewer->spinOnce(1000 / frame_per_sec);
		frame_count++;
		time_us = 1000000 * frame_count / frame_per_sec;
	}

	// NIS Metric preparation begin
	plt::xkcd();
	const double chi_square_95 = 7.815;
	std::vector<double> critical_line(frame_per_sec * sec_interval, chi_square_95);
	plt::figure_size(1280, 720);
	plt::named_plot("95% Line", critical_line, "r--");
	plt::named_plot("Radar NIS", highway.traffic_nis_radar);
	plt::named_plot("Lidar NIS", highway.traffic_nis_lidar);
	plt::legend();
	plt::save("./NIS_UKF.png");
	plt::show();
	// NIS Metric preparation end
}