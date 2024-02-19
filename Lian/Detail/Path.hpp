#pragma once

#include <vector>

#include "../Point.hpp"

namespace Algorithms {

	namespace Graph {

		namespace Geometry {

			struct Path {
				std::vector<Point> points;

				double distance;
				double sumAngles;

				Path(std::vector<Point> points, double distance, double sumAngles)
					: points(points), distance(distance), sumAngles(sumAngles) {}
			};
		}
	}
}