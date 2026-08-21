namespace HIGHOMEGA
{
	namespace GEOM
	{
		struct EdgeUV
		{
			vec3 e1, e2;
			vec2 uv1, uv2;
		};
		void AddEdge(std::vector <EdgeUV> & edges, EdgeUV edge);
		void MirrorTrianglesAndHoleFill(std::vector <TriUVTanBasis> & triList, vec3 separatingPlaneNorm, vec3 separatingPlanePt);
		void FindAndGenerateOOBB(std::vector <TriUVTanBasis> & triList, mat4 & separationPlane, vec3 &OOBB, std::vector <TriUVTanBasis> & oobbTris);
		void CloseLoopConcave(std::vector<EdgeUV> & edges, std::vector <TriUVTanBasis> & triList, vec3 & hitPoint);
	}
}
