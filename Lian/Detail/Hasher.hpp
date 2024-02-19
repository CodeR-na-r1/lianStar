#pragma once

#include <unordered_set>

#include "StagePoint.hpp"

namespace Hasher {

	using Algorithms::Graph::Geometry::StagePoint;

	struct StagePointHasher {

		template <class T>
		inline void hash_combine(size_t& seed, const T& v) const noexcept {
			std::hash<T> hasher;
			seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		}

		size_t operator()(const StagePoint& sP) const noexcept {

			size_t res{ 0 };

			hash_combine<int>(res, sP.point.x);
			hash_combine<int>(res, sP.point.y);

			hash_combine<int>(res, sP.parent.x);
			hash_combine<int>(res, sP.parent.y);

			//hash_combine<double>(res, sP.distance);
			//hash_combine<double>(res, sP.sumAngles);

			return res;
		}
	};

}