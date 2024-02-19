#pragma once

#include "../Point.hpp"

namespace Algorithms {

	namespace Graph {

		namespace Geometry {

			struct StagePoint {
				Point point;
				Point parent;

				double distance;
				double sumAngles;

				StagePoint(Point point_, Point parent_, double distance_, double sumAngles_)
					: point(point_), parent(parent_), distance(distance_), sumAngles(sumAngles_) {};

				auto operator<=>(const StagePoint& rhs) const = default;
			};

		}
	}
}