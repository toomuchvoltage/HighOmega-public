unsigned int RigidBody::RR_NativeIDX = -1;
RadeonRays::IntersectionApi *RigidBody::RR_api = nullptr;

void HIGHOMEGA::FIZ_X::RigidBody::GenRadeonRaysShape()
{
	//choose device
	if (RR_NativeIDX == -1)
	{
		// Always use OpenCL
		RadeonRays::IntersectionApi::SetPlatform(RadeonRays::DeviceInfo::kOpenCL);

		for (auto idx = 0U; idx < RadeonRays::IntersectionApi::GetDeviceCount(); ++idx)
		{
			RadeonRays::DeviceInfo devinfo;
			RadeonRays::IntersectionApi::GetDeviceInfo(idx, devinfo);

			if (devinfo.type == RadeonRays::DeviceInfo::kGpu && RR_NativeIDX == -1)
			{
				RR_NativeIDX = idx;
				break;
			}
		}
		if (RR_NativeIDX == -1) throw ("No GPU device found for RadeonRays");
		RR_api = RadeonRays::IntersectionApi::Create(RR_NativeIDX);
	}

	TimerObject tRR;
	tRR.Start();

	std::vector<float> allVerts;
	std::vector<int> allIndices;

	int triCounter = 0;
	for (ObjectPiece & curPiece : this->pieces)
		for (ObjectTriFacet & curTri : curPiece.tris)
			triCounter++;

	allVerts.reserve(triCounter * 9);
	allIndices.reserve(triCounter * 3);

	triCounter = 0;

	for (ObjectPiece & curPiece : this->pieces)
	{
		for (ObjectTriFacet & curTri : curPiece.tris)
		{
			int e1 = curTri.e1;
			int e2 = curTri.e2;
			int e3 = curTri.e3;

			vec3 v1 = curPiece.verts_tran[e1];
			vec3 v2 = curPiece.verts_tran[e2];
			vec3 v3 = curPiece.verts_tran[e3];

			allVerts.push_back(v1.x);
			allVerts.push_back(v1.y);
			allVerts.push_back(v1.z);

			allVerts.push_back(v2.x);
			allVerts.push_back(v2.y);
			allVerts.push_back(v2.z);

			allVerts.push_back(v3.x);
			allVerts.push_back(v3.y);
			allVerts.push_back(v3.z);

			allIndices.push_back(triCounter);
			triCounter++;
			allIndices.push_back(triCounter);
			triCounter++;
			allIndices.push_back(triCounter);
			triCounter++;
		}
	}
	RR_shape = RR_api->CreateMesh(allVerts.data(), (unsigned int)allVerts.size() / 3, 3 * sizeof(float), allIndices.data(), 0, nullptr, (unsigned int)allIndices.size() / 3);
	if (RR_shape == nullptr) throw ("Could not create RadeonRays shape");
	RR_api->AttachShape(RR_shape);
	RR_api->Commit();
	LOG() << "Radeon Rays build time: " << tRR.Diff();
}

void HIGHOMEGA::FIZ_X::RigidBody::GetIntersections(unsigned int width, unsigned int height)
{
	float invW = 1.0f / ((float)width);
	float invH = 1.0f / ((float)height);
	if ( RR_rays.size() == 0 ) RR_rays.resize(width * height);
	for (int i = 0; i != width; i++)
		for (int j = 0; j != height; j++)
		{
			vec2 coordNorm = vec2 ((float)i * invW, (float)j * invH) * 2.0f - vec2 (1.0f);
			vec3 rayDir = vec3(coordNorm.x, coordNorm.y, 1.0f).normalized();
			RadeonRays::ray newRay;
			newRay.o = RadeonRays::float4(0.0f, 0.0f, 0.0f, 1000.0f);
			newRay.d = RadeonRays::float3(-rayDir.x, -rayDir.y, rayDir.z);
			RR_rays[j * width + i] = newRay;
		}

	TimerObject isectTimer;
	isectTimer.Start();
	auto ray_buffer = RR_api->CreateBuffer((unsigned int)RR_rays.size() * sizeof(RadeonRays::ray), RR_rays.data());
	auto isect_buffer = RR_api->CreateBuffer((width * height) * sizeof(RadeonRays::Intersection), nullptr);
	RR_api->QueryIntersection(ray_buffer, width * height, isect_buffer, nullptr, nullptr);
	RadeonRays::Event* e = nullptr;
	RadeonRays::Intersection* tmp = nullptr;
	RR_api->MapBuffer(isect_buffer, RadeonRays::kMapRead, 0, (width * height) * sizeof(RadeonRays::Intersection), (void**)&tmp, &e);
	e->Wait();
	RR_api->DeleteEvent(e);
	e = nullptr;
	LOG() << "RadeonRays isect time: " << isectTimer.Diff();

	if (RR_res == nullptr) RR_res = new unsigned char[width * height * 4];

	for (int i = 0; i != width; i++)
		for (int j = 0; j != height; j++)
		{
			unsigned int curIndex = j * width + i;
			unsigned int curPixel = curIndex * 4;
			if (tmp[curIndex].primid == -1)
			{
				RR_res[curPixel] = (unsigned char)255;
				RR_res[curPixel + 1] = (unsigned char)0;
				RR_res[curPixel + 2] = (unsigned char)0;
				RR_res[curPixel + 3] = (unsigned char)255;
			}
			else
			{
				float darkAmount = (vec3(RR_rays[curIndex].d.x, RR_rays[curIndex].d.y, RR_rays[curIndex].d.z) * 1000.0f * tmp[curIndex].uvwt.w).length() * 0.00001f;
				unsigned char charVal = (unsigned char)(min ((unsigned int)(darkAmount * 255.0f), 255));
				RR_res[curPixel] = charVal;
				RR_res[curPixel + 1] = charVal;
				RR_res[curPixel + 2] = charVal;
				RR_res[curPixel + 3] = (unsigned char)255;
			}
		}
}


void * HIGHOMEGA::FIZ_X::RigidBody::getBVHTriangles()
{
	return BVHTris;
}

unsigned int HIGHOMEGA::FIZ_X::RigidBody::getBVHTrianglesMemSize()
{
	return BVH.getNumTris() * sizeof (BVHTriangle);
}

void * HIGHOMEGA::FIZ_X::RigidBody::getBVHInternalNodes()
{
	return BVH.getInternalNodes();
}

unsigned int HIGHOMEGA::FIZ_X::RigidBody::getBVHInternalNodesMemSize()
{
	return (BVH.getNumTris() * 2 - 1) * sizeof(BVHInternalNode);
}

void HIGHOMEGA::FIZ_X::RigidBody::GenBVH()
{
	if (BVHTris) return;
	if (!BVHTris)
	{
		unsigned int totalTriCount = 0;
		for (ObjectPiece & curPiece : this->pieces)
			totalTriCount += (unsigned int)curPiece.tris.size();

		BVHTris = new BVHTriangle[totalTriCount];
	}
	unsigned int triCounter = 0;

	for (ObjectPiece & curPiece : this->pieces)
	{
		ImageClass & diffImg = TextureCache[curPiece.diffName].elem;
		std::string nrmName = curPiece.diffName;
		nrmName.replace(nrmName.rfind(".tga"), std::string(".tga").length(), ".nrm.tga");
		std::string rghName = curPiece.diffName;
		rghName.replace(rghName.rfind(".tga"), std::string(".tga").length(), ".rgh.tga");

		vec2 diffCoords = diffImg.textureAtlasCoords;
		vec2 nrmCoords = vec2(0.0);
		vec2 rghCoords = vec2(0.0);

		unsigned int diffWidth = diffImg.getWidth();
		unsigned int diffHeight = diffImg.getHeight();
		unsigned int nrmWidth = 0;
		unsigned int nrmHeight = 0;
		unsigned int rghWidth = 0;
		unsigned int rghHeight = 0;

		if (TextureCache.find(nrmName) != TextureCache.end()) {
			ImageClass & nrmImg = TextureCache[nrmName].elem;
			nrmCoords = nrmImg.textureAtlasCoords;
			nrmWidth = nrmImg.getWidth();
			nrmHeight = nrmImg.getHeight();
		}

		if (TextureCache.find(rghName) != TextureCache.end()) {
			ImageClass & rghImg = TextureCache[rghName].elem;
			rghCoords = rghImg.textureAtlasCoords;
			rghWidth = rghImg.getWidth();
			rghHeight = rghImg.getHeight();
		}

		for (ObjectTriFacet & curTri : curPiece.tris)
		{
			int e1 = curTri.e1;
			int e2 = curTri.e2;
			int e3 = curTri.e3;
			int n = curTri.n;
			int t = curTri.t;
			int b = curTri.b;
			int s1 = curTri.s1;
			int s2 = curTri.s2;
			int s3 = curTri.s3;

			vec3 v1 = curPiece.verts_tran[e1];
			vec3 v2 = curPiece.verts_tran[e2];
			vec3 v3 = curPiece.verts_tran[e3];
			vec3 norm = curPiece.nts_tran[n];
			vec3 tan = curPiece.nts_tran[t];
			vec3 bitan = curPiece.nts_tran[b];

			BVHTris[triCounter].e1MinComp[0] = v1.x;
			BVHTris[triCounter].e1MinComp[1] = v1.y;
			BVHTris[triCounter].e1MinComp[2] = v1.z;
			BVHTris[triCounter].e1MinComp[3] = curTri.min.x;

			BVHTris[triCounter].e2MinComp[0] = v2.x;
			BVHTris[triCounter].e2MinComp[1] = v2.y;
			BVHTris[triCounter].e2MinComp[2] = v2.z;
			BVHTris[triCounter].e2MinComp[3] = curTri.min.y;

			BVHTris[triCounter].e3MinComp[0] = v3.x;
			BVHTris[triCounter].e3MinComp[1] = v3.y;
			BVHTris[triCounter].e3MinComp[2] = v3.z;
			BVHTris[triCounter].e3MinComp[3] = curTri.min.z;

			BVHTris[triCounter].nMaxComp[0] = norm.x;
			BVHTris[triCounter].nMaxComp[1] = norm.y;
			BVHTris[triCounter].nMaxComp[2] = norm.z;
			BVHTris[triCounter].nMaxComp[3] = curTri.max.x;

			BVHTris[triCounter].tMaxComp[0] = tan.x;
			BVHTris[triCounter].tMaxComp[1] = tan.y;
			BVHTris[triCounter].tMaxComp[2] = tan.z;
			BVHTris[triCounter].tMaxComp[3] = curTri.max.y;

			BVHTris[triCounter].bMaxComp[0] = bitan.x;
			BVHTris[triCounter].bMaxComp[1] = bitan.y;
			BVHTris[triCounter].bMaxComp[2] = bitan.z;
			BVHTris[triCounter].bMaxComp[3] = curTri.max.z;

			BVHTris[triCounter].diffUVCoords[0] = (unsigned int)diffCoords.x;
			BVHTris[triCounter].diffUVCoords[1] = (unsigned int)diffCoords.y;
			BVHTris[triCounter].diffUVCoords[2] = (unsigned int)diffWidth;
			BVHTris[triCounter].diffUVCoords[3] = (unsigned int)diffHeight;
			BVHTris[triCounter].normUVCoords[0] = (unsigned int)nrmCoords.x;
			BVHTris[triCounter].normUVCoords[1] = (unsigned int)nrmCoords.y;
			BVHTris[triCounter].normUVCoords[2] = (unsigned int)nrmWidth;
			BVHTris[triCounter].normUVCoords[3] = (unsigned int)nrmHeight;
			BVHTris[triCounter].roughUVCoords[0] = (unsigned int)rghCoords.x;
			BVHTris[triCounter].roughUVCoords[1] = (unsigned int)rghCoords.y;
			BVHTris[triCounter].roughUVCoords[2] = (unsigned int)rghWidth;
			BVHTris[triCounter].roughUVCoords[3] = (unsigned int)rghHeight;
			BVHTris[triCounter].uv1uv2[0] = curTri.uv1.x;
			BVHTris[triCounter].uv1uv2[1] = curTri.uv1.y;
			BVHTris[triCounter].uv1uv2[2] = curTri.uv2.x;
			BVHTris[triCounter].uv1uv2[3] = curTri.uv2.y;
			BVHTris[triCounter].uv3[0] = curTri.uv3.x;
			BVHTris[triCounter].uv3[1] = curTri.uv3.y;
			BVHTris[triCounter].dielectricAndIoR = curPiece.dielectricAndIoR;
			BVHTris[triCounter].emissivity = curPiece.emissivity;

			triCounter++;
		}
	}

	BVH.ProduceInternalNodes(BVHTris, triCounter);
}

HIGHOMEGA::FIZ_X::RigidBody::RigidBody(std::string & newGroupId, ObjectPiece & origPiece, std::vector<TriUVTanBasis>& triList, vec3 inpNewCenter) : meshRef (nullptr)
{
	...

	BVHTris = nullptr;

	...
	
    newPiece.dielectricAndIoR = origPiece.dielectricAndIoR;
    newPiece.emissivity = origPiece.emissivity;
}

HIGHOMEGA::FIZ_X::RigidBody::RigidBody(Mesh & inpMesh, std::string belong , mat4 inpOrient, vec3 inpPos, bool inIsMap, std::function<bool(int, DataGroup &)> inpFilterFunction, vec3 *inpNewCenter) : meshRef(&inpMesh)
{
	BVHTris = nullptr;

	...
	
	float IoRFetch = 0.0f, diElectricFetch = 0.0f;
	try
	{
		IoRFetch = Mesh::getDataRowFloat(curPolyGroup, "PROPS", "refractiveIndex");
	}
	catch (...)
	{
	}
	try
	{
		diElectricFetch += Mesh::getDataRowFloat(curPolyGroup, "PROPS", "dielectric");
	}
	catch (...)
	{
	}
	curPiece.dielectricAndIoR = (IoRFetch * 0.1f) + (diElectricFetch * 1.0f);

	try
	{
		curPiece.emissivity = Mesh::getDataRowFloat(curPolyGroup, "PROPS", "emissivity");
	}
	catch (...)
	{
		curPiece.emissivity = 0.0f;
	}

	...
}

void HIGHOMEGA::FIZ_X::RigidBody::ChangeGeom(std::string & groupId, std::vector <TriUVTanBasis> & triList)
{
	...
	
    newPiece.dielectricAndIoR = oldPiece->dielectricAndIoR;
    newPiece.emissivity = oldPiece->emissivity;
	
	...
}

HIGHOMEGA::FIZ_X::RigidBody::~RigidBody()
{
	if (BVHTris) delete[] BVHTris;
	BVHTris = nullptr;
}
