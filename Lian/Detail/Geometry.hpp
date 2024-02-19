#pragma once

#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>

#include "../Point.hpp"

namespace Algorithms {

	namespace Graph {

		namespace Geometry {

			double distanceBetweenPoints(Point p1, Point p2) {

				return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
			}

			double angleBetweenVectors(Point a1, Point a2, Point b1, Point b2) {

				double ax = a2.x - a1.x;
				double ay = a2.y - a1.y;
				double bx = b2.x - b1.x;
				double by = b2.y - b1.y;

				double angle = atan2(by, bx) - atan2(ay, ax);

				return abs(angle * 180 / M_PI);
			}

			std::vector<Point> lineOfSight(Point p1, Point p2) {

				int nb_points = distanceBetweenPoints(p1, p2);

				double x_spacing = (p2.x - p1.x) / double(nb_points + 1);
				double y_spacing = (p2.y - p1.y) / double(nb_points + 1);

				std::vector<Point> points;
				for (int i = 1; i < nb_points + 1; ++i) {

					points.emplace_back(int(p1.x + i * x_spacing), int(p1.y + i * y_spacing));
				}

				return points;
			}

			std::vector<Point> midpoint(Point point, int r) {

				std::vector<Point> points;
				int x_centre = point.x;
				int y_centre = point.y;

				int x = r;
				int y = 0;
				points.emplace_back(x + x_centre, y + y_centre);
				if (r > 0) {
					points.emplace_back(-x + x_centre, -y + y_centre);
					points.emplace_back(y + x_centre, x + y_centre);
					points.emplace_back(-y + x_centre, -x + y_centre);
				}

				int P = 1 - r;

				while (x > y) {

					y += 1;
					if (P <= 0) {
						P = P + 2 * y + 1;
					}
					else {
						x -= 1;
						P = P + 2 * y - 2 * x + 1;
					}

					if (x < y)
						break;

					points.emplace_back(x + x_centre, y + y_centre);
					points.emplace_back(-x + x_centre, y + y_centre);
					points.emplace_back(x + x_centre, -y + y_centre);
					points.emplace_back(-x + x_centre, -y + y_centre);

					if (x != y) {
						points.emplace_back(y + x_centre, x + y_centre);;
						points.emplace_back(-y + x_centre, x + y_centre);
						points.emplace_back(y + x_centre, -x + y_centre);
						points.emplace_back(-y + x_centre, -x + y_centre);
					}


				}

				return points;
			}

		}
	}
}