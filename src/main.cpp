#include "highway.h"

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
}