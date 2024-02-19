#pragma once

#include "../Point.hpp"

namespace Algorithms {

	namespace Graph {

		namespace Map {

			using Algorithms::Graph::Geometry::Point;

			template <typename T>
			class Map {
				T image;

			public:

				Map(T img) :image(img) {}

				cv::Mat getMap() {

					return image;
				}

				bool isFree(Point point) {

					auto res = (int)image.at<uchar>(cv::Point(point.x, point.y));
					return res != 0;
					//return ((int)image.at<uchar>(cv::Point(point.x, point.y))) != 0;
				}

			};
		}
	}
}