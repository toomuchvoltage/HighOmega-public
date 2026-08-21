layout (binding = 5, r32ui) uniform readonly uimage2D diffuseTrace

...

vec3 getEncodingBasis3(vec3 norm)
{
	if (abs(norm) != vec3(0.0, 1.0, 0.0))
		return normalize (cross(norm, vec3(0.0, 1.0, 0.0)));
	else
		return normalize (cross(vec3(1.0, 0.0, 0.0), norm));
}

void getEncodingBasis(vec3 norm, out vec3 basis1, out vec3 basis2, out vec3 basis3, out vec3 basis4)
{
	if (abs(norm) != vec3(0.0, 1.0, 0.0))
	{
		basis3 = normalize (cross(norm, vec3(0.0, 1.0, 0.0)));
		basis1 = cross(basis3, norm);
		basis2 = -basis1;
		basis4 = -basis3;
	}
	else
	{
		basis3 = normalize (cross(vec3(1.0, 0.0, 0.0), norm));
		basis1 = cross(norm, basis3);
		basis2 = -basis1;
		basis4 = -basis3;
	}
}

void addDiffuseIrradiance(ivec2 loc, vec3 irrad, vec3 rayDir, vec3 pos, vec3 norm)
{
	uint prevOffset, curOffset;
	bool turnEven = (pathTraceParams.diffuseMVPTurnGlossTrailTurn >> 4) == 0;
	if (turnEven)
	{
		curOffset = loc.x * 28;
		prevOffset = curOffset + 14;
	}
	else
	{
		prevOffset = loc.x * 28;
		curOffset = prevOffset + 14;
	}

	float prevX = uintBitsToFloat (imageLoad(diffuseTrace, ivec2(prevOffset    , loc.y)).x);
	float prevY = uintBitsToFloat (imageLoad(diffuseTrace, ivec2(prevOffset + 1, loc.y)).x);
	float prevZ = uintBitsToFloat (imageLoad(diffuseTrace, ivec2(prevOffset + 2, loc.y)).x);
	vec3 prevPos = vec3 (prevX, prevY, prevZ);
	vec3 prevNorm = fromZSignXY (imageLoad(diffuseTrace, ivec2(prevOffset + 3, loc.y)).x);

	vec4 prevIrrad[5] = vec4[](vec4(0.0), vec4(0.0), vec4(0.0), vec4(0.0), vec4(0.0));

	vec3 normBasis3 = getEncodingBasis3(norm);
	if ( length (prevPos - pos) < 10.0 && dot (prevNorm, norm) > 0.9 && dot (getEncodingBasis3(prevNorm), normBasis3) > 0.9 )
	{
		prevIrrad[0] = vec4 (unpackHalf2x16(imageLoad(diffuseTrace, ivec2(prevOffset + 4 , loc.y)).x), unpackHalf2x16(imageLoad(diffuseTrace, ivec2(prevOffset + 5 , loc.y)).x));
		prevIrrad[1] = vec4 (unpackHalf2x16(imageLoad(diffuseTrace, ivec2(prevOffset + 6 , loc.y)).x), unpackHalf2x16(imageLoad(diffuseTrace, ivec2(prevOffset + 7 , loc.y)).x));
		prevIrrad[2] = vec4 (unpackHalf2x16(imageLoad(diffuseTrace, ivec2(prevOffset + 8 , loc.y)).x), unpackHalf2x16(imageLoad(diffuseTrace, ivec2(prevOffset + 9 , loc.y)).x));
		prevIrrad[3] = vec4 (unpackHalf2x16(imageLoad(diffuseTrace, ivec2(prevOffset + 10, loc.y)).x), unpackHalf2x16(imageLoad(diffuseTrace, ivec2(prevOffset + 11, loc.y)).x));
		prevIrrad[4] = vec4 (unpackHalf2x16(imageLoad(diffuseTrace, ivec2(prevOffset + 12, loc.y)).x), unpackHalf2x16(imageLoad(diffuseTrace, ivec2(prevOffset + 13, loc.y)).x));
	}
	else
	{
		vec2 posPrevUV = getPrevScreenSampleCoord (pos);
		if (clamp (posPrevUV, vec2(0.0), vec2(0.999999)) == posPrevUV)
		{
			ivec2 probesResolution = imageSize (diffuseTrace);
			probesResolution.x /= 28;
			ivec2 posPrevLoc = ivec2 (vec2 (probesResolution) * posPrevUV);
			uint posPrevLocPrevOffset;
			posPrevLocPrevOffset = turnEven ? posPrevLoc.x * 28 + 14 : posPrevLoc.x * 28;

			float posPrevLocPrevX = uintBitsToFloat (imageLoad(diffuseTrace, ivec2(posPrevLocPrevOffset    , posPrevLoc.y)).x);
			float posPrevLocPrevY = uintBitsToFloat (imageLoad(diffuseTrace, ivec2(posPrevLocPrevOffset + 1, posPrevLoc.y)).x);
			float posPrevLocPrevZ = uintBitsToFloat (imageLoad(diffuseTrace, ivec2(posPrevLocPrevOffset + 2, posPrevLoc.y)).x);
			vec3 posPrevLocPrevPos = vec3 (posPrevLocPrevX, posPrevLocPrevY, posPrevLocPrevZ);
			vec3 posPrevLocPrevNorm = fromZSignXY (imageLoad(diffuseTrace, ivec2(posPrevLocPrevOffset + 3, posPrevLoc.y)).x);
			
			if ( length (posPrevLocPrevPos - pos) < 10.0 && dot (posPrevLocPrevNorm, norm) > 0.9 && dot (getEncodingBasis3(posPrevLocPrevNorm), normBasis3) > 0.9 )
			{
				prevIrrad[0] = vec4 (unpackHalf2x16(imageLoad(diffuseTrace, ivec2(posPrevLocPrevOffset + 4 , posPrevLoc.y)).x), unpackHalf2x16(imageLoad(diffuseTrace, ivec2(posPrevLocPrevOffset + 5 , posPrevLoc.y)).x));
				prevIrrad[1] = vec4 (unpackHalf2x16(imageLoad(diffuseTrace, ivec2(posPrevLocPrevOffset + 6 , posPrevLoc.y)).x), unpackHalf2x16(imageLoad(diffuseTrace, ivec2(posPrevLocPrevOffset + 7 , posPrevLoc.y)).x));
				prevIrrad[2] = vec4 (unpackHalf2x16(imageLoad(diffuseTrace, ivec2(posPrevLocPrevOffset + 8 , posPrevLoc.y)).x), unpackHalf2x16(imageLoad(diffuseTrace, ivec2(posPrevLocPrevOffset + 9 , posPrevLoc.y)).x));
				prevIrrad[3] = vec4 (unpackHalf2x16(imageLoad(diffuseTrace, ivec2(posPrevLocPrevOffset + 10, posPrevLoc.y)).x), unpackHalf2x16(imageLoad(diffuseTrace, ivec2(posPrevLocPrevOffset + 11, posPrevLoc.y)).x));
				prevIrrad[4] = vec4 (unpackHalf2x16(imageLoad(diffuseTrace, ivec2(posPrevLocPrevOffset + 12, posPrevLoc.y)).x), unpackHalf2x16(imageLoad(diffuseTrace, ivec2(posPrevLocPrevOffset + 13, posPrevLoc.y)).x));
			}
		}
	}

	vec3 basis1, basis2, basis3, basis4;
	getEncodingBasis(norm, basis1, basis2, basis3, basis4);
	float weightNorm   = clamp (dot(norm  , rayDir), 0.0, 1.0);
	float weightBasis1 = clamp (dot(basis1, rayDir), 0.0, 1.0);
	float weightBasis2 = clamp (dot(basis2, rayDir), 0.0, 1.0);
	float weightBasis3 = clamp (dot(basis3, rayDir), 0.0, 1.0);
	float weightBasis4 = clamp (dot(basis4, rayDir), 0.0, 1.0);
	prevIrrad[0] += vec4 (irrad * weightNorm  , weightNorm   * 0.5);
	prevIrrad[1] += vec4 (irrad * weightBasis1, weightBasis1 * 0.5);
	prevIrrad[2] += vec4 (irrad * weightBasis2, weightBasis2 * 0.5);
	prevIrrad[3] += vec4 (irrad * weightBasis3, weightBasis3 * 0.5);
	prevIrrad[4] += vec4 (irrad * weightBasis4, weightBasis4 * 0.5);
	imageStore (diffuseTrace, ivec2(curOffset     , loc.y), uvec4(floatBitsToUint(pos.x), 0, 0, 0));
	imageStore (diffuseTrace, ivec2(curOffset + 1 , loc.y), uvec4(floatBitsToUint(pos.y), 0, 0, 0));
	imageStore (diffuseTrace, ivec2(curOffset + 2 , loc.y), uvec4(floatBitsToUint(pos.z), 0, 0, 0));
	imageStore (diffuseTrace, ivec2(curOffset + 3 , loc.y), uvec4(toZSignXY(norm), 0, 0, 0));
	imageStore (diffuseTrace, ivec2(curOffset + 4 , loc.y), uvec4(packHalf2x16(prevIrrad[0].xy), 0, 0, 0));
	imageStore (diffuseTrace, ivec2(curOffset + 5 , loc.y), uvec4(packHalf2x16(prevIrrad[0].zw), 0, 0, 0));
	imageStore (diffuseTrace, ivec2(curOffset + 6 , loc.y), uvec4(packHalf2x16(prevIrrad[1].xy), 0, 0, 0));
	imageStore (diffuseTrace, ivec2(curOffset + 7 , loc.y), uvec4(packHalf2x16(prevIrrad[1].zw), 0, 0, 0));
	imageStore (diffuseTrace, ivec2(curOffset + 8 , loc.y), uvec4(packHalf2x16(prevIrrad[2].xy), 0, 0, 0));
	imageStore (diffuseTrace, ivec2(curOffset + 9 , loc.y), uvec4(packHalf2x16(prevIrrad[2].zw), 0, 0, 0));
	imageStore (diffuseTrace, ivec2(curOffset + 10, loc.y), uvec4(packHalf2x16(prevIrrad[3].xy), 0, 0, 0));
	imageStore (diffuseTrace, ivec2(curOffset + 11, loc.y), uvec4(packHalf2x16(prevIrrad[3].zw), 0, 0, 0));
	imageStore (diffuseTrace, ivec2(curOffset + 12, loc.y), uvec4(packHalf2x16(prevIrrad[4].xy), 0, 0, 0));
	imageStore (diffuseTrace, ivec2(curOffset + 13, loc.y), uvec4(packHalf2x16(prevIrrad[4].zw), 0, 0, 0));
}

...

vec3 getDiffuseRadiance(vec2 curUV, vec3 pos, vec3 norm)
{
	vec4 retVal = vec4 (0.0);
	ivec2 probesResolution = imageSize (diffuseTrace);
	probesResolution.x /= 28;

	bool turnEven = (pathTraceParams.diffuseMVPTurnGlossTrailTurn >> 4) == 0;
	ivec2 curLoc = ivec2 (curUV * vec2(probesResolution));
	
	vec3 normBasis3 = getEncodingBasis3(norm);

	for (int i = -5; i != 6; i++)
		for (int j = -5; j != 6; j++)
		{
			ivec2 sampleLoc = curLoc + ivec2(i, j);
			if (clamp (sampleLoc, ivec2(0), probesResolution - ivec2(1)) != sampleLoc) continue;
			uint curOffset = turnEven ? sampleLoc.x * 28 : sampleLoc.x * 28 + 14;

			float readPosX = uintBitsToFloat (imageLoad(diffuseTrace, ivec2(curOffset    , sampleLoc.y)).x);
			float readPosY = uintBitsToFloat (imageLoad(diffuseTrace, ivec2(curOffset + 1, sampleLoc.y)).x);
			float readPosZ = uintBitsToFloat (imageLoad(diffuseTrace, ivec2(curOffset + 2, sampleLoc.y)).x);
			vec3 readPos = vec3 (readPosX, readPosY, readPosZ);
			vec3 readNorm = fromZSignXY (imageLoad(diffuseTrace, ivec2(curOffset + 3, sampleLoc.y)).x);

			float distCheck = length(readPos - pos);
			float normCheck = dot(readNorm, norm);
			float basisCheck = dot(getEncodingBasis3(readNorm), normBasis3);
			if ( distCheck < 30.0 && normCheck > 0.0 && basisCheck > 0.0 )
			{
				vec4 readIrrad0 = vec4 (unpackHalf2x16(imageLoad(diffuseTrace, ivec2(curOffset + 4 , sampleLoc.y)).x), unpackHalf2x16(imageLoad(diffuseTrace, ivec2(curOffset + 5 , sampleLoc.y)).x));
				vec4 readIrrad1 = vec4 (unpackHalf2x16(imageLoad(diffuseTrace, ivec2(curOffset + 6 , sampleLoc.y)).x), unpackHalf2x16(imageLoad(diffuseTrace, ivec2(curOffset + 7 , sampleLoc.y)).x));
				vec4 readIrrad2 = vec4 (unpackHalf2x16(imageLoad(diffuseTrace, ivec2(curOffset + 8 , sampleLoc.y)).x), unpackHalf2x16(imageLoad(diffuseTrace, ivec2(curOffset + 9 , sampleLoc.y)).x));
				vec4 readIrrad3 = vec4 (unpackHalf2x16(imageLoad(diffuseTrace, ivec2(curOffset + 10, sampleLoc.y)).x), unpackHalf2x16(imageLoad(diffuseTrace, ivec2(curOffset + 11, sampleLoc.y)).x));
				vec4 readIrrad4 = vec4 (unpackHalf2x16(imageLoad(diffuseTrace, ivec2(curOffset + 12, sampleLoc.y)).x), unpackHalf2x16(imageLoad(diffuseTrace, ivec2(curOffset + 13, sampleLoc.y)).x));
				vec3 basis1, basis2, basis3, basis4;
				getEncodingBasis(readNorm, basis1, basis2, basis3, basis4);
				float similarityWeight = min(basisCheck, basisCheck) * (1.0 - (distCheck/30.0)) * clamp (1.0 - (length(vec2(i, j)) / length(vec2(5.0))), 0.0, 1.0);
				retVal += clamp (dot(norm, readNorm), 0.0, 1.0) * readIrrad0 * similarityWeight;
				retVal += clamp (dot(norm, basis1  ), 0.0, 1.0) * readIrrad1 * similarityWeight;
				retVal += clamp (dot(norm, basis2  ), 0.0, 1.0) * readIrrad2 * similarityWeight;
				retVal += clamp (dot(norm, basis3  ), 0.0, 1.0) * readIrrad3 * similarityWeight;
				retVal += clamp (dot(norm, basis4  ), 0.0, 1.0) * readIrrad4 * similarityWeight;
			}
		}

	if (retVal.a > 0.0) retVal.rgb /= retVal.a;

	return retVal.rgb;
}