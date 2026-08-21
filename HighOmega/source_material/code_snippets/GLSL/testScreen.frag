#version 460

#define VOXEL_DIM_SIZE gridInfo.mapMinVoxelSize.w

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_nonuniform_qualifier : require

#define M_PI 3.1415926535

layout (binding = 1) uniform testPropsUBO
{
    float time;
    float res1;
    float res2;
    float res3;
} testProps;

/*layout (binding = 2) uniform GridInfoUBO
{
	vec4 mapMinVoxelSize;
	vec4 mapMaxReserved;
} gridInfo;

layout (binding = 3, rgba8) uniform readonly image3D voxelizedScene;*/
layout (binding = 2, rgba8) uniform readonly image2D ptOutput;
layout (binding = 3) uniform sampler2D onScreenText;
layout (binding = 4) uniform sampler2D fontMap;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;
layout (location = 2) in vec3 inEye;

layout (location = 0) out vec4 diffOutput;

vec4 getText()
{
	if ( gl_FragCoord.x > 720 || gl_FragCoord.y > 100 ) return vec4(0.0); // 60 * 12, 5 * 20

	ivec2 tCoord = ivec2 (gl_FragCoord.xy * vec2 (0.08333333333, 0.05)); // 1/12.0, 1/20.0
	uint charNum = uint (texelFetch (onScreenText, tCoord.xy, 0).x * 255.0);
	
	vec2 startUV = vec2 (charNum % 16, 15 - (charNum / 16)) * 0.0625; // 1.0/16.0
	
	vec4 textVal = texture (fontMap, startUV + vec2(int (gl_FragCoord.x) % 12, 19 - int(gl_FragCoord.y) % 20) * vec2 (0.00520833333, 0.003125)) * 1.2; // 1/(16.0 * 12.0), 1/(16.0 * 20.0)
	vec4 backDropVal = vec4 (0.0, 0.0, 0.0, 0.5);
	return mix (backDropVal, textVal, textVal.a);
}

vec3 toVec3(vec2 inpVec)
{
	float phi = inpVec.x * (2.0 * M_PI);
	float theta = inpVec.y * M_PI;
	vec3 onPlane = vec3 (cos(phi), 0.0, 0.0) + vec3 (0.0, 0.0, sin(phi));
	return onPlane*sin(theta) + vec3(0.0, cos(theta), 0.0);
}

void main()
{
	/*vec2 dirXYComp = (inUV - vec2 (0.5)) * 4.0;
	vec3 boxLen = gridInfo.mapMaxReserved.xyz - gridInfo.mapMinVoxelSize.xyz;
	vec3 invBoxLen = 1.0 / boxLen;
	
	vec3 gridSizeFloat = ceil(boxLen / VOXEL_DIM_SIZE);

	vec3 ourEye = vec3 (cos (testProps.time *  0.1), 0.0, sin (testProps.time *  0.1)) * 10.0;
	vec3 ourLook = normalize (-ourEye);
	vec3 ourUp = vec3 (0.0, 1.0, 0.0);
	vec3 ourSide = cross (ourLook, ourUp);
	vec3 pixelDir = normalize (ourLook - ourUp * dirXYComp.y + ourSide * dirXYComp.x);
	
	vec3 curPos = ourEye + pixelDir;
	
	for (int w = 0; w != 1000; w++)
	{
		vec3 posUVW = (curPos - gridInfo.mapMinVoxelSize.xyz)*invBoxLen;
		if ( posUVW != clamp (posUVW, vec3 (0.0), vec3 (0.999999)) ) break;
		ivec3 queryLoc = ivec3 (posUVW * gridSizeFloat);
		queryLoc.x *= 8;
		vec4 diffFetch = imageLoad (voxelizedScene, queryLoc);
		if ( diffFetch.a == 0.0 )
		{
			queryLoc.x += 4;
			diffFetch = imageLoad (voxelizedScene, queryLoc);
		}
		if ( diffFetch.a != 0.0 )
		{
			diffOutput = diffFetch;
			return ;
		}
		curPos += pixelDir;
	}
	diffOutput = vec4 (1.0, 0.0, 0.0, 1.0);*/
	
	ivec2 decodedTexelLocation = ivec2 (inUV * vec2 (imageSize(ptOutput).xy));

	//diffOutput = unpackUnorm4x8 (floatBitsToUint (texelFetch (diffuseAttach, decodedTexelLocation, 0).x));
	//diffOutput = vec4 (toVec3 (texelFetch (diffuseAttach, decodedTexelLocation, 0).xy), 1.0);
	diffOutput = imageLoad (ptOutput, decodedTexelLocation);
	vec4 textVal = getText();
	diffOutput = mix (diffOutput, textVal, textVal.a);
}