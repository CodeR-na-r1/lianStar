#pragma once

namespace Algorithms {

	namespace Graph {

		namespace Geometry {

			struct Point {

				int x, y;

				auto operator<=>(const Point& rhs) const = default;
			};
		}
	}
}