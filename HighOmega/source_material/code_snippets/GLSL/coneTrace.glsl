// For testing cone tracing sometimes
vec3 testConeTrace(vec3 startRay, vec3 startDir, float refractiveIndex, vec3 startNorm, vec3 startGeomNorm, vec3 startTan, vec3 startBiTan, vec2 startRoughness, float startSpecularity, vec3 seed1)
{
	vec3 storeLight = vec3 (0.0);
	for (int i = 0;i != 21;i++)
		for (int j = 1;j != 6;j++)
		{
			float phi =  (float(i)/21.0) * M_PI * 2.0;
			float theta = (float(j)/6.0) * M_PI * 0.5;
			vec3 curPos = startRay + startGeomNorm * VOXEL_DIM_SIZE * 1.41;
			vec3 dirToTrace = (cos(phi) * startTan + sin(phi) * startBiTan) * cos (theta) + startNorm * sin (theta);
			while (true)
			{
				vec3 uvwLoc = getUVW (curPos);
				if ( uvwLoc == vec3 (2.0) )
				{
					storeLight += abs (dot (startNorm, dirToTrace)) * getSkyValue (dirToTrace) * (1.0/100.0);
					break;
				}
				vec4 diffFetch = texture(preIlluminateImage, uvwLoc);
				if ( diffFetch.a >= 1.0 )
				{
					storeLight += abs (dot (startNorm, dirToTrace)) * diffFetch.rgb * (1.0/100.0);
					break ;
				}
				curPos += dirToTrace * getDistance (uvwLoc) * VOXEL_DIM_SIZE;
			}
		}
		
	return storeLight;
}