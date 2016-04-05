#include "awesomo/controller.hpp"


CarrotController::CarrotController() : initialized(0) { }

CarrotController::CarrotController(
    std::deque<Eigen::Vector3d> waypoints,
    double look_ahead_dist,
    double wp_threshold
)
{
    this->initialized = 1;
    this->waypoints = waypoints;
    this->look_ahead_dist = look_ahead_dist;
    this->wp_threshold = wp_threshold;
}

Eigen::Vector3d CarrotController::closestPoint(
    Eigen::Vector3d position,
    Eigen::Vector3d wp_start,
    Eigen::Vector3d wp_end
)
{
    double t;
    Eigen::Vector3d v1;
    Eigen::Vector3d v2;

	// calculate closest point
	v1 = position - wp_start;
	v2 = wp_end - wp_start;
	t = v1.dot(v2) / v2.squaredNorm();

    // make sure the point is between wp_start and wp_end
	if (t < 0) {
		return wp_start;
    } else if (t > 1) {
		return wp_end;
    }

    // result
    return wp_start + t * v2;
}

Eigen::Vector3d CarrotController::calculateCarrotPoint(
    Eigen::Vector3d position,
    double r,
    Eigen::Vector3d wp_start,
    Eigen::Vector3d wp_end
)
{
    Eigen::Vector3d u;
    Eigen::Vector3d v;
    Eigen::Vector3d pt_on_line;

    // get closest point
    pt_on_line = this->closestPoint(position, wp_start, wp_end);

    // calculate carrot point on wp_start and wp_end
    v = wp_end - wp_start;
    u = v / v.norm();
    return pt_on_line + r * u;
}

int CarrotController::waypointReached(
    Eigen::Vector3d position,
    Eigen::Vector3d waypoint,
    double threshold
)
{
    double dist;
    Eigen::Vector3d x;

    // calculate distance to waypoint
    x = waypoint - position;
    dist = x.norm();

    // waypoint reached?
    if (dist > threshold) {
        return 0;
    } else {
        return 1;
    }
}

int CarrotController::update(Eigen::Vector3d position, Eigen::Vector3d &carrot)
{
    int nb_waypoints;

    // pre-check
    if (this->initialized == 0) {
        return -2;
    }

    // waypoint reached? get new wp_start and wp_end
    if (this->waypointReached(position, wp_end, 1)) {
        if (this->waypoints.size() > 2) {
            this->waypoints.pop_front();
            this->wp_start = this->waypoints.at(0);
            this->wp_end = this->waypoints.at(1);
        } else {
            return 0;
        }
    }

    // calculate new carrot
    carrot = this->calculateCarrotPoint(
        position,
        this->look_ahead_dist,
        this->wp_start,
        this->wp_end
    );

    return 1;
}
