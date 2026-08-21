#include "radeonrays/radeon_rays.h"
#include "accelstructs.h"

using namespace HIGHOMEGA::MATH::ACCEL_STRUCT;

			/* this section is only for BVH */
		public:
			BVHGenClass BVH;
			BVHTriangle *BVHTris;
			void *getBVHTriangles();
			unsigned int getBVHTrianglesMemSize();
			void *getBVHInternalNodes();
			unsigned int getBVHInternalNodesMemSize();
			void GenBVH();
			/* end of BVH section */

			/* this section is only for RadeonRays */
		public:
			static unsigned int RR_NativeIDX;
			static RadeonRays::IntersectionApi* RR_api;
			unsigned char *RR_res = nullptr;
			RadeonRays::Shape* RR_shape = nullptr;
			std::vector <RadeonRays::ray> RR_rays;
			void GenRadeonRaysShape();
			void GetIntersections(unsigned int width, unsigned int height);
			/* end of RadeonRays section */