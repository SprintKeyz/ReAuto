// This code is ported from LemLib (I really like its path generator).
// check it out at https://github.com/LemLib/LemLib

#include "reauto/motion/purepursuit/PurePursuit.hpp"
#include "reauto/datatypes/Pose.h"
#include <cmath>
#include <vector>
#include <string>

namespace reauto {
namespace motion {
PurePursuit::PurePursuit(MotionChassis* chassis) : m_chassis(chassis) {}

std::vector<std::string> readElement(const std::string& input, std::string delimiter) {
	std::string token;
	std::string s = input;
	std::vector<std::string> output;
	size_t pos = 0;

	// main loop
	while ((pos = s.find(delimiter)) != std::string::npos) { // while there are still delimiters in the string
		token = s.substr(0, pos); // processed substring
		output.push_back(token);
		s.erase(0, pos + delimiter.length()); // remove the read substring
	}

	output.push_back(s); // add the last element to the returned string

	return output;
}

std::vector<Pose> getData(const char* path) {
	std::vector<Pose> robotPath;
	std::string line;
	std::vector<std::string> pointInput;
	Pose pathPoint = { 0, 0, 0 };

	FILE* usd_path_file = fopen(path, "r");

	// get file size
	fseek(usd_path_file, 0, SEEK_END);
	long fileSize = ftell(usd_path_file);
	rewind(usd_path_file);

	// malloc for file data
	char* data = (char*)malloc(fileSize);

	// read file data into buffer
	fread(data, 1, fileSize, usd_path_file);

	// format data from the asset
	std::vector<std::string> dataLines = readElement(data, "\n");

	// read the points until 'endData' is read
	for (std::string line : dataLines) {
		if (line == "endData" || line == "endData\r") break;
		pointInput = readElement(line, ", "); // parse line
		pathPoint.x = std::atof(pointInput.at(0).c_str()); // x position
		pathPoint.y = std::atof(pointInput.at(1).c_str()); // y position
		pathPoint.theta = std::atof(pointInput.at(2).c_str()); // velocity
		robotPath.push_back(pathPoint); // save data
	}

	fclose(usd_path_file);
	free(data);

	return robotPath;
}

int findClosest(Pose pose, std::vector<Pose> path) {
	int closestPoint;
	float closestDist = 1000000;
	float dist;

	// loop through all path points
	for (int i = 0; i < path.size(); i++) {
		dist = calc::distance({ pose.x, pose.y }, { path.at(i).x, path.at(i).y });
		if (dist < closestDist) { // new closest point
			closestDist = dist;
			closestPoint = i;
		}
	}

	return closestPoint;
}

// TODO: remove this in favor of calc::lineCircleIntersect
float circleIntersect(Pose p1, Pose p2, Pose pose, float lookaheadDist) {
	// calculations
	// uses the quadratic formula to calculate intersection points
	Pose d = p2 - p1;
	Pose f = p1 - pose;
	float a = d * d;
	float b = 2 * (f * d);
	float c = (f * f) - lookaheadDist * lookaheadDist;
	float discriminant = b * b - 4 * a * c;

	// if a possible intersection was found
	if (discriminant >= 0) {
		discriminant = sqrt(discriminant);
		float t1 = (-b - discriminant) / (2 * a);
		float t2 = (-b + discriminant) / (2 * a);

		// prioritize further down the path
		if (t2 >= 0 && t2 <= 1) return t2;
		else if (t1 >= 0 && t1 <= 1) return t1;
	}

	// no intersection found
	return -1;
}

Pose lookaheadPoint(Pose lastLookahead, Pose pose, std::vector<Pose> path,
	float lookaheadDist) {
	// find the furthest lookahead point on the path

	// optimizations applied:
	// - made the starting index the one after lastLookahead's index,
	// as anything before would be discarded
	// - searched the path in reverse, as the first hit would be
	// the garunteed farthest lookahead point
	for (int i = path.size() - 1; i > lastLookahead.theta; i--) {
		// since we are searching in reverse, instead of getting
		// the current pose and the next one, we should get the
		// current pose and the *last* one
		Pose lastPathPose = path.at(i - 1);
		Pose currentPathPose = path.at(i);

		float t = circleIntersect(lastPathPose, currentPathPose, pose, lookaheadDist);

		if (t != -1) {
			Point lookaheadPoint = calc::lerp({ lastPathPose.x, lastPathPose.y }, { currentPathPose.x, currentPathPose.y }, t);
			Pose lookahead = { lookaheadPoint.x, lookaheadPoint.y, i };
			return lookahead;
		}
	}

	// robot deviated from path, use last lookahead point
	return lastLookahead;
}

int sgn(double x) {
	if (x > 0) return 1;
	else return -1;
}

float findLookaheadCurvature(Pose pose, float heading, Pose lookahead) {
	// calculate whether the robot is on the left or right side of the circle
	float side = sgn(std::sin(heading) * (lookahead.x - pose.x) - std::cos(heading) * (lookahead.y - pose.y));
	// calculate center point and radius
	float a = -std::tan(heading);
	float c = std::tan(heading) * pose.x - pose.y;
	float x = std::fabs(a * lookahead.x + lookahead.y + c) / std::sqrt((a * a) + 1);
	float d = std::hypot(lookahead.x - pose.x, lookahead.y - pose.y);

	// return curvature
	return side * ((2 * x) / (d * d));
}

// remove the "false" from getPose if there are issues
void PurePursuit::follow(const char* path, int timeout, double lookahead, bool reverse, double maxSpeed) {
	std::vector<Pose> pathPoints = getData(path);
	Pose pose = m_chassis->getPose(true, false);
	Pose lastPose = pose;
	Pose lookaheadPose = { 0, 0, 0 };
	Pose lastLookahead = pathPoints.at(0);
	lastLookahead.theta = 0;

	double curvature;
	double targetVel;
	double prevLeftVel = 0;
	double prevRightVel = 0;
	int closestPoint;
	double leftInput = 0;
	double rightInput = 0;

	m_distTraveled = 0;

	// loop until robot within end tolerance
	for (int i = 0; i < timeout / 10; i++) {
		pose = m_chassis->getPose(true, false);

		// if reversed, flip the pose
		if (reverse) pose.theta = pose.theta.value_or(0.0) - M_PI;

		// update completion vars
		m_distTraveled += calc::distance({ pose.x, pose.y }, { lastPose.x, lastPose.y });
		lastPose = pose;

		// find the closest point to the robot
		closestPoint = findClosest(pose, pathPoints);

		// if robot is at the end of the path, break
		if (pathPoints.at(closestPoint).theta == 0) break;

		// find the lookahead point
		lookaheadPose = lookaheadPoint(lastLookahead, pose, pathPoints, lookahead);
		lastLookahead = lookaheadPose;

		// calc curavture of the arc to the lookahead point
		double curvatureHeading = M_PI / 2 - pose.theta.value_or(0.0);
		curvature = findLookaheadCurvature(pose, curvatureHeading, lookaheadPose);

		// calculate target velocity
		targetVel = pathPoints.at(closestPoint).theta.value_or(0.0);

		// calculate target LR velocities
		double targetLeftVel = targetVel * (2 + curvature * m_chassis->getMeasurements().trackWidth) / 2;
		double targetRightVel = targetVel * (2 - curvature * m_chassis->getMeasurements().trackWidth) / 2;

		// ratio the speeds to the max speed
		double ratio = std::max(std::fabs(targetLeftVel), std::fabs(targetRightVel)) / maxSpeed;
		if (ratio > 1) {
			targetLeftVel /= ratio;
			targetRightVel /= ratio;
		}

		// update prev velocities
		prevLeftVel = targetLeftVel;
		prevRightVel = targetRightVel;

		// move the robot
		if (reverse) {
			m_chassis->getLeftMotors().move(-targetLeftVel);
			m_chassis->getRightMotors().move(-targetRightVel);
		}
		else {
			m_chassis->getLeftMotors().move(targetLeftVel);
			m_chassis->getRightMotors().move(targetRightVel);
		}

		pros::delay(10);
	}

	// stop the robot
	m_chassis->getLeftMotors().brake();
	m_chassis->getRightMotors().brake();

	// reset members
	// -1 indicates the function has finished
	m_distTraveled = -1;
}
}
}