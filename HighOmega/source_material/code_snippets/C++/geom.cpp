void HIGHOMEGA::GEOM::AddEdge(std::vector<EdgeUV>& edges, EdgeUV edge)
{
	float tolerance = 0.0001f;
	for (int i = 0; i != edges.size(); i++) {
		if ((WithinAABB(edges[i].e1, edge.e1, tolerance) && WithinAABB(edges[i].e2, edge.e2, tolerance)) ||
			(WithinAABB(edges[i].e2, edge.e1, tolerance) && WithinAABB(edges[i].e1, edge.e2, tolerance))) {
			edges.erase(edges.begin() + i);
			return;
		}
	}
	edges.push_back(edge);
}

void HIGHOMEGA::GEOM::MirrorTrianglesAndHoleFill(std::vector<TriUVTanBasis>& triList, vec3 separatingPlaneNorm, vec3 separatingPlanePt)
{
	std::vector <TriUVTanBasis> mirroredTris;
	triList.reserve(triList.size() * 7);
	mirroredTris.reserve(triList.size() * 6);
	vec3 eArr[3];
	vec2 uvArr[3];
	for (TriUVTanBasis & curTri : triList)
	{
		vec3 refEArr[3];
		vec2 refUVArr[3];
		refEArr[0] = ReflectPos(curTri.eArr[0], separatingPlaneNorm, separatingPlanePt);
		refEArr[1] = ReflectPos(curTri.eArr[1], separatingPlaneNorm, separatingPlanePt);
		refEArr[2] = ReflectPos(curTri.eArr[2], separatingPlaneNorm, separatingPlanePt);
		vec3 normVec = ReflectDir(curTri.normVec, separatingPlaneNorm);
		vec3 tanVec = ReflectDir(curTri.tanVec, separatingPlaneNorm);
		vec3 bitanVec = ReflectDir(curTri.bitanVec, separatingPlaneNorm);
		AddTri(refEArr, curTri.uvArr, normVec, tanVec, bitanVec, mirroredTris);

		for (int j = 0; j != 3; j++) {
			eArr[0] = curTri.eArr[j];
			eArr[1] = curTri.eArr[(j + 1) % 3];
			eArr[2] = refEArr[j];
			uvArr[0] = curTri.uvArr[j];
			uvArr[1] = curTri.uvArr[(j + 1) % 3];
			uvArr[2] = curTri.uvArr[j];
			vec3 newNorm = cross(curTri.eArr[0] - curTri.eArr[1], refEArr[0] - curTri.eArr[1]).normalized();
			if (newNorm * (curTri.eArr[(j + 2) % 3] - curTri.eArr[0]) > 0.0f) newNorm = -newNorm;
			vec3 newBiTan = cross(newNorm, curTri.tanVec);
			AddTri(eArr, uvArr, normVec, curTri.tanVec, newBiTan, mirroredTris);
			eArr[0] = curTri.eArr[(j + 1) % 3];
			eArr[1] = refEArr[j];
			eArr[2] = refEArr[(j + 1) % 3];
			uvArr[0] = curTri.uvArr[(j + 1) % 3];
			uvArr[1] = curTri.uvArr[j];
			uvArr[2] = curTri.uvArr[(j + 1) % 3];
			AddTri(eArr, uvArr, normVec, curTri.tanVec, newBiTan, mirroredTris);
		}
	}
	triList.insert(triList.end(), mirroredTris.begin(), mirroredTris.end());
}

void HIGHOMEGA::GEOM::FindAndGenerateOOBB(std::vector<TriUVTanBasis>& triList, mat4 & separationPlane, vec3 & OOBB, std::vector <TriUVTanBasis> & oobbTris)
{
	oobbTris.clear();

	vec3 X = vec3(separationPlane.i[0][0], separationPlane.i[1][0], separationPlane.i[2][0]);
	vec3 Y = vec3(separationPlane.i[0][1], separationPlane.i[1][1], separationPlane.i[2][1]);
	vec3 Z = vec3(separationPlane.i[0][2], separationPlane.i[1][2], separationPlane.i[2][2]);
	vec3 placePoint = vec3(separationPlane.i[0][3], separationPlane.i[1][3], separationPlane.i[2][3]);

	for (int i = 0; i != triList.size(); i++)
	{
		for (int j = 0; j != 3; j++)
		{
			vec3 subVec = triList[i].eArr[j] - placePoint;
			float xShadow = fabs(subVec * X);
			float yShadow = fabs(subVec * Y);
			float zShadow = fabs(subVec * Z);
			if (i == 0 && j == 0)
				OOBB = vec3(xShadow, yShadow, zShadow);
			else
			{
				OOBB = vec3(max(xShadow, OOBB.x), max(yShadow, OOBB.y), max(zShadow, OOBB.z));
			}
		}
	}

	TriUVTanBasis curTri;
	vec3 constantAxis, changeAxis1, changeAxis2;
	vec3 shadowValues;
	for (int i = 0; i != 6; i++)
	{
		switch (i)
		{
		case 0:
			constantAxis = X;
			changeAxis1 = Y;
			changeAxis2 = Z;
			shadowValues = OOBB;
			break;
		case 1:
			constantAxis = -X;
			changeAxis1 = Y;
			changeAxis2 = Z;
			break;
		case 2:
			constantAxis = Y;
			changeAxis1 = X;
			changeAxis2 = Z;
			shadowValues = vec3(OOBB.y, OOBB.x, OOBB.z);
			break;
		case 3:
			constantAxis = -Y;
			changeAxis1 = X;
			changeAxis2 = Z;
			break;
		case 4:
			constantAxis = Z;
			changeAxis1 = X;
			changeAxis2 = Y;
			shadowValues = vec3(OOBB.z, OOBB.x, OOBB.y);
			break;
		case 5:
			constantAxis = -Z;
			changeAxis1 = X;
			changeAxis2 = Y;
			break;
		}
		curTri.eArr[0] = placePoint + constantAxis * shadowValues.x + changeAxis1 * shadowValues.y + changeAxis2 * shadowValues.z;
		curTri.eArr[1] = placePoint + constantAxis * shadowValues.x - changeAxis1 * shadowValues.y + changeAxis2 * shadowValues.z;
		curTri.eArr[2] = placePoint + constantAxis * shadowValues.x - changeAxis1 * shadowValues.y - changeAxis2 * shadowValues.z;
		curTri.uvArr[0] = vec2(1.0f);
		curTri.uvArr[1] = vec2(0.0f, 1.0f);
		curTri.uvArr[2] = vec2(0.0f, 0.0f);
		curTri.normVec = constantAxis;
		GetTangentBiTangent(curTri.eArr, curTri.uvArr, curTri.tanVec, curTri.bitanVec);
		AddTri(curTri.eArr, curTri.uvArr, curTri.normVec, curTri.tanVec, curTri.bitanVec, oobbTris);
		curTri.eArr[1] = placePoint + constantAxis * shadowValues.x + changeAxis1 * shadowValues.y - changeAxis2 * shadowValues.z;
		curTri.uvArr[1] = vec2(1.0f, 0.0f);
		GetTangentBiTangent(curTri.eArr, curTri.uvArr, curTri.tanVec, curTri.bitanVec);
		AddTri(curTri.eArr, curTri.uvArr, curTri.normVec, curTri.tanVec, curTri.bitanVec, oobbTris);
	}
}

void HIGHOMEGA::GEOM::CloseLoopConcave(std::vector<EdgeUV>& edges, std::vector<TriUVTanBasis>& triList, vec3 & hitPoint)
{
	int loopingCount = 0;
	float tolerance = 0.001f;
	while (edges.size() > 0) {
		unsigned int edgeSizeBeforeChange = (unsigned int)edges.size();
		vec3 e1;
		vec3 e2;
		vec3 e3;
		vec3 e4;
		vec2 e1uv;
		vec2 e2uv;
		vec2 e3uv;
		vec2 e4uv;
		bool joined;
		vec3 normVec1;
		vec3 normVec2;
		vec3 candidate_e1;
		vec3 candidate_e2;
		vec3 candidate_e3;
		vec3 candidate_e4;
		vec3 candidate_Norm1;
		vec3 candidate_Norm2;
		vec2 candidate_e1uv;
		vec2 candidate_e2uv;
		vec2 candidate_e3uv;
		vec2 candidate_e4uv;
		bool candidate_joined;
		float maxDist = 0.0f;
		for (int i = 0; i != edges.size(); i++) {
			for (int j = 0; j != edges.size(); j++) {
				if (i == j) continue;
				bool WithinAABB_je1_ie1 = WithinAABB(edges[j].e1, edges[i].e1, tolerance);
				bool WithinAABB_je2_ie1 = WithinAABB(edges[j].e2, edges[i].e1, tolerance);
				bool WithinAABB_je1_ie2 = WithinAABB(edges[j].e1, edges[i].e2, tolerance);
				bool WithinAABB_je2_ie2 = WithinAABB(edges[j].e2, edges[i].e2, tolerance);
				if (WithinAABB_je1_ie1 || WithinAABB_je2_ie1 || WithinAABB_je1_ie2 || WithinAABB_je2_ie2) {
					candidate_joined = true;
					if (WithinAABB_je1_ie1 || WithinAABB_je1_ie2) {
						candidate_e1 = edges[i].e1;
						candidate_e2 = edges[i].e2;
						candidate_e3 = edges[j].e2;
						candidate_e1uv = edges[i].uv1;
						candidate_e2uv = edges[i].uv2;
						candidate_e3uv = edges[j].uv2;
					}
					else {
						candidate_e1 = edges[i].e1;
						candidate_e2 = edges[i].e2;
						candidate_e3 = edges[j].e1;
						candidate_e1uv = edges[i].uv1;
						candidate_e2uv = edges[i].uv2;
						candidate_e3uv = edges[j].uv1;
					}
					candidate_Norm1 = cross(candidate_e1 - candidate_e2, candidate_e3 - candidate_e2).normalized();
					vec3 hitPoint_ce2 = hitPoint - candidate_e2;
					float distToHitPoint1 = candidate_Norm1 * hitPoint_ce2;
					if (distToHitPoint1 < 0.0f) candidate_Norm1 = -candidate_Norm1;
					float distToCandidate = fabs(distToHitPoint1);
					if (distToCandidate > maxDist) {
						maxDist = distToCandidate;
						e1 = candidate_e1;
						e2 = candidate_e2;
						e3 = candidate_e3;
						e1uv = candidate_e1uv;
						e2uv = candidate_e2uv;
						e3uv = candidate_e3uv;
						normVec1 = candidate_Norm1;
						joined = candidate_joined;
					}
				}
				else
				{
					candidate_e1 = edges[i].e1;
					candidate_e2 = edges[i].e2;
					candidate_e3 = edges[j].e1;
					candidate_e4 = edges[j].e2;
					candidate_e1uv = edges[i].uv1;
					candidate_e2uv = edges[i].uv2;
					candidate_e3uv = edges[j].uv1;
					candidate_e4uv = edges[j].uv2;
					candidate_joined = false;
					candidate_Norm1 = cross(candidate_e1 - candidate_e2, candidate_e3 - candidate_e2).normalized();
					candidate_Norm2 = cross(candidate_e2 - candidate_e3, candidate_e4 - candidate_e3).normalized();
					vec3 hitPoint_ce2 = hitPoint - candidate_e2;
					float distToHitPoint1 = candidate_Norm1 * hitPoint_ce2;
					if (distToHitPoint1 < 0.0f) candidate_Norm1 = -candidate_Norm1;
					float distToHitPoint2 = candidate_Norm2 * hitPoint_ce2;
					if (distToHitPoint2 < 0.0f) candidate_Norm2 = -candidate_Norm2;
					float distToCandidate = min(fabs(distToHitPoint1), fabs(distToHitPoint2));
					if (distToCandidate > maxDist) {
						maxDist = distToCandidate;
						e1 = candidate_e1;
						e2 = candidate_e2;
						e3 = candidate_e3;
						e4 = candidate_e4;
						e1uv = candidate_e1uv;
						e2uv = candidate_e2uv;
						e3uv = candidate_e3uv;
						e4uv = candidate_e4uv;
						normVec1 = candidate_Norm1;
						normVec2 = candidate_Norm2;
						joined = candidate_joined;
					}
				}
			}
		}

		vec3 eArr[3];
		vec2 uvArr[3];
		vec3 tanVec, bitanVec;
		if (joined)
		{
			eArr[0] = e1;
			eArr[1] = e2;
			eArr[2] = e3;
			uvArr[0] = e1uv;
			uvArr[1] = e2uv;
			uvArr[2] = e3uv;
			GetTangentBiTangent(eArr, uvArr, tanVec, bitanVec);
			AddTri(eArr, uvArr, normVec1, tanVec, bitanVec, triList);
			EdgeUV newEdge;
			newEdge.e1 = e1;
			newEdge.e2 = e2;
			newEdge.uv1 = e1uv;
			newEdge.uv2 = e2uv;
			AddEdge(edges, newEdge);
			newEdge.e1 = e2;
			newEdge.e2 = e3;
			newEdge.uv1 = e2uv;
			newEdge.uv2 = e3uv;
			AddEdge(edges, newEdge);
			newEdge.e1 = e3;
			newEdge.e2 = e1;
			newEdge.uv1 = e3uv;
			newEdge.uv2 = e1uv;
			AddEdge(edges, newEdge);
		}
		else
		{
			eArr[0] = e1;
			eArr[1] = e2;
			eArr[2] = e3;
			uvArr[0] = e1uv;
			uvArr[1] = e2uv;
			uvArr[2] = e3uv;
			GetTangentBiTangent(eArr, uvArr, tanVec, bitanVec);
			AddTri(eArr, uvArr, normVec1, tanVec, bitanVec, triList);
			eArr[0] = e2;
			eArr[1] = e3;
			eArr[2] = e4;
			uvArr[0] = e2uv;
			uvArr[1] = e3uv;
			uvArr[2] = e4uv;
			GetTangentBiTangent(eArr, uvArr, tanVec, bitanVec);
			AddTri(eArr, uvArr, normVec2, tanVec, bitanVec, triList);
			EdgeUV newEdge;
			newEdge.e1 = e1;
			newEdge.e2 = e2;
			newEdge.uv1 = e1uv;
			newEdge.uv2 = e2uv;
			AddEdge(edges, newEdge);
			newEdge.e1 = e3;
			newEdge.e2 = e4;
			newEdge.uv1 = e3uv;
			newEdge.uv2 = e1uv;
			AddEdge(edges, newEdge);
			newEdge.e1 = e3;
			newEdge.e2 = e1;
			newEdge.uv1 = e3uv;
			newEdge.uv2 = e1uv;
			AddEdge(edges, newEdge);
			newEdge.e1 = e2;
			newEdge.e2 = e4;
			newEdge.uv1 = e3uv;
			newEdge.uv2 = e1uv;
			AddEdge(edges, newEdge);
		}

		if (edges.size() == edgeSizeBeforeChange) loopingCount++;
		if (loopingCount == 100) return;
		if (edges.size() > edgeSizeBeforeChange) return; // We can't find a solution... give up
	}
}