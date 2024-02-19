#pragma once

#include "../Point.hpp"
#include "StagePoint.hpp"

#include "Geometry.hpp"

namespace Comparator {

	using Algorithms::Graph::Geometry::Point;
	using Algorithms::Graph::Geometry::StagePoint;
	using Algorithms::Graph::Geometry::distanceBetweenPoints;

	Point goal = Point(0, 0);
	float KDelta = 1.0;
	float KAngle = 0.0;

	struct ComparatorStagePoint {

		bool operator()(const StagePoint& lhs, const StagePoint& rhs) const {

			return (lhs.distance + distanceBetweenPoints(lhs.point, goal)) * KDelta
				+ (lhs.sumAngles) * KAngle
				<
				(rhs.distance + distanceBetweenPoints(rhs.point, goal)) * KDelta
				+ (rhs.sumAngles) * KAngle;
		}
	};
}