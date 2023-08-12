/*
	Copyright (c) 2023 TooMuchVoltage Software Inc.

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

#include <noise.h>

HIGHOMEGA::MATH::NOISE::Noise::Noise(unsigned int w, unsigned int h, unsigned int d)
{
	width = w;
	height = h;
	depth = d;
}

vec3 HIGHOMEGA::MATH::NOISE::PerlinNoise::getGrad(int inX, int inY, int inZ)
{
	unsigned int gradLoc = (inZ * width * height + inY * width + inX) * 3;
	float gradX = PerlinGrad[gradLoc];
	float gradY = PerlinGrad[gradLoc + 1];
	float gradZ = PerlinGrad[gradLoc + 2];
	return vec3(gradX, gradY, gradZ);
}

void HIGHOMEGA::MATH::NOISE::PerlinNoise::putGrad(int inX, int inY, int inZ, vec3 inpGrad)
{
	unsigned int gradLoc = (inZ * width * height + inY * width + inX) * 3;
	PerlinGrad[gradLoc] = inpGrad.x;
	PerlinGrad[gradLoc + 1] = inpGrad.y;
	PerlinGrad[gradLoc + 2] = inpGrad.z;
}

float HIGHOMEGA::MATH::NOISE::PerlinNoise::evalPerlinNoise(vec3 inpCoord)
{
	double coordXInt;
	double coordYInt;
	double coordZInt;
	float coordXFract = (float)modf(inpCoord.x, &coordXInt);
	float coordYFract = (float)modf(inpCoord.y, &coordYInt);
	float coordZFract = (float)modf(inpCoord.z, &coordZInt);

	float coordXFractSmooth = SmootherstepFast(coordXFract);
	float coordYFractSmooth = SmootherstepFast(coordYFract);
	float coordZFractSmooth = SmootherstepFast(coordZFract);

	int coordXIntCast = (int)(coordXInt) % (width - 1);
	int coordYIntCast = (int)(coordYInt) % (height - 1);
	int coordZIntCast = (int)(coordZInt) % (depth - 1);
	int coordX1IntCast = coordXIntCast + 1;
	int coordY1IntCast = coordYIntCast + 1;
	int coordZ1IntCast = coordZIntCast + 1;
	vec3 fractVec = vec3 (coordXFract, coordYFract, coordZFract);
	vec3 fractMinusOneVec = vec3(coordXFract, coordYFract, coordZFract) - vec3 (1.0f);

	float dotGradient000 = getGrad(coordXIntCast, coordYIntCast, coordZIntCast) * fractVec;
	float dotGradient010 = getGrad(coordXIntCast, coordY1IntCast, coordZIntCast) * vec3 (fractVec.x, fractMinusOneVec.y, fractVec.z);
	float dotGradient100 = getGrad(coordX1IntCast, coordYIntCast, coordZIntCast) * vec3(fractMinusOneVec.x, fractVec.y, fractVec.z);
	float dotGradient110 = getGrad(coordX1IntCast, coordY1IntCast, coordZIntCast) * vec3(fractMinusOneVec.x, fractMinusOneVec.y, fractVec.z);

	float dotGradient001 = getGrad(coordXIntCast, coordYIntCast, coordZ1IntCast) * vec3(fractVec.x, fractVec.y, fractMinusOneVec.z);
	float dotGradient011 = getGrad(coordXIntCast, coordY1IntCast, coordZ1IntCast) * vec3(fractVec.x, fractMinusOneVec.y, fractMinusOneVec.z);
	float dotGradient101 = getGrad(coordX1IntCast, coordYIntCast, coordZ1IntCast) * vec3(fractMinusOneVec.x, fractVec.y, fractMinusOneVec.z);
	float dotGradient111 = getGrad(coordX1IntCast, coordY1IntCast, coordZ1IntCast) * fractMinusOneVec;

	return Lerp(
		Lerp(
			Lerp(dotGradient000, dotGradient010, coordYFractSmooth),
			Lerp(dotGradient100, dotGradient110, coordYFractSmooth), coordXFractSmooth),

		Lerp(
			Lerp(dotGradient001, dotGradient011, coordYFractSmooth),
			Lerp(dotGradient101, dotGradient111, coordYFractSmooth), coordXFractSmooth),

		coordZFractSmooth);
}

void HIGHOMEGA::MATH::NOISE::PerlinNoise::RemovePast()
{
	if (PerlinGrad) delete PerlinGrad;

	PerlinGrad = nullptr;
}

HIGHOMEGA::MATH::NOISE::PerlinNoise::PerlinNoise(unsigned int w, unsigned int h, unsigned int d, vec3 inpBaseSt, unsigned int inpNumOctaves) : Noise(w, h, d)
{
	PerlinGrad = nullptr;

	baseSt = inpBaseSt;
	numOctaves = inpNumOctaves;

	PerlinGrad = new float[width * height * depth * 3];

	for (int i = 0; i != width; i++)
		for (int j = 0; j != height; j++)
			for (int k = 0; k != depth; k++)
				putGrad(i, j, k, (vec3((float)(rand() % 200 - 100), (float)(rand() % 200 - 100), (float)(rand() % 200 - 100)) * 0.01f).normalized());
}

HIGHOMEGA::MATH::NOISE::PerlinNoise::~PerlinNoise()
{
	RemovePast();
}

float HIGHOMEGA::MATH::NOISE::PerlinNoise::evalNoise(vec3 inpCoord)
{
	vec3 inpAdjustedCoord = vec3(inpCoord.x * baseSt.x, inpCoord.y * baseSt.y, inpCoord.z * baseSt.z);
	float retVal = 0.0f;

	if (numOctaves == 0)
	{
		retVal = evalPerlinNoise(inpAdjustedCoord);
	}
	else
	{
		float amplitude = 0.5f;
		for (unsigned int co = 0; co != numOctaves; co++) {
			retVal += amplitude * evalPerlinNoise(inpAdjustedCoord);
			inpAdjustedCoord *= 2.0f;
			amplitude *= 0.5f;
		}
	}

	return (retVal + 1.0f) * 0.5f;
}

void HIGHOMEGA::MATH::NOISE::WorleyNoise::RemovePast()
{
	if (bins) delete bins;

	bins = nullptr;
}

HIGHOMEGA::MATH::NOISE::WorleyNoise::WorleyNoise(unsigned int w, unsigned int h, unsigned int d, unsigned int inpBinDim, bool inpReverse, unsigned int inpNumOctaves) : Noise(w, h, d)
{
	bins = nullptr;
	reverse = inpReverse;
	numOctaves = inpNumOctaves;

	if (inpBinDim == 0) throw std::runtime_error("Invalid bin size");

	binDim = inpBinDim;

	binW = (int)ceil(((float)w) / ((float)binDim));
	binH = (int)ceil(((float)h) / ((float)binDim));
	binD = (int)ceil(((float)d) / ((float)binDim));

	bins = new vec3[binW * binH * binD];

	for (int i = 0;i != binW;i++)
		for (int j = 0; j != binH; j++)
			for (int k = 0; k != binD; k++)
				bins[k * binW * binH + j * binW + i] = vec3((float)(rand() % 100), (float)(rand() % 100), (float)(rand() % 100)) * 0.01f;
}

HIGHOMEGA::MATH::NOISE::WorleyNoise::~WorleyNoise()
{
	RemovePast();
}

float HIGHOMEGA::MATH::NOISE::WorleyNoise::evalNoise(vec3 inpCoord)
{
	float retVal = 0.0f;
	if (numOctaves == 0)
	{
		retVal = evalNoiseWorley(inpCoord);
	}
	else
	{
		float amplitude = 0.5f;
		for (unsigned int co = 0; co != numOctaves; co++) {
			retVal += amplitude * evalNoiseWorley(inpCoord);
			inpCoord *= 2.0f;
			amplitude *= 0.5f;
		}
	}
	return retVal;
}

float HIGHOMEGA::MATH::NOISE::WorleyNoise::evalNoiseWorley(vec3 inpCoord)
{
	vec3 inpAdjustedCoord = inpCoord / ((float)binDim);

	double coordXInt;
	double coordYInt;
	double coordZInt;
	float coordXFract = (float)modf(inpAdjustedCoord.x, &coordXInt);
	float coordYFract = (float)modf(inpAdjustedCoord.y, &coordYInt);
	float coordZFract = (float)modf(inpAdjustedCoord.z, &coordZInt);

	int coordXIntMinus1 = (int)(coordXInt - 1.0);
	int coordYIntMinus1 = (int)(coordYInt - 1.0);
	int coordZIntMinus1 = (int)(coordZInt - 1.0);
	int coordXIntPlus1 = coordXIntMinus1 + 2;
	int coordYIntPlus1 = coordYIntMinus1 + 2;
	int coordZIntPlus1 = coordZIntMinus1 + 2;

	float minDist = 1.0f;
	float invBinDim = 1.0f / ((float)binDim);
	for (int i = coordXIntMinus1;i <= coordXIntPlus1; i++)
		for (int j = coordYIntMinus1; j <= coordYIntPlus1; j++)
			for (int k = coordZIntMinus1; k <= coordZIntPlus1; k++)
			{
				if (i < 0 || i >= binW || j < 0 || j >= binH || k < 0 || k >= binD) continue;
				vec3 curPos = (vec3((float)i, (float)j, (float)k) + bins[k * binW * binH + j * binW + i]) * ((float)binDim);
				float curDist = min ((curPos - inpCoord).length() * invBinDim, 1.0f);
				if (curDist < minDist) minDist = curDist;
			}

	return reverse ? 1.0f - minDist : minDist;
}

void HIGHOMEGA::MATH::NOISE::NoiseGenerator::FillNoiseThread(unsigned int width, unsigned int height, unsigned int depth, Noise **allNoisesArray, unsigned char *vals, int threadNum, int maxThreads)
{
	for (int i = 0; i != width; i++)
		for (int j = 0; j != height; j++)
			for (int k = 0; k != depth; k++)
			{
				if (k % maxThreads != threadNum) continue;

				float noiseRed = allNoisesArray[0]->evalNoise(vec3 ((float)i, (float)j, (float)k));
				float noiseGreen = allNoisesArray[1]->evalNoise(vec3((float)i, (float)j, (float)k));
				float noiseBlue = allNoisesArray[2]->evalNoise(vec3((float)i, (float)j, (float)k));
				float noiseAlpha = allNoisesArray[3]->evalNoise(vec3((float)i, (float)j, (float)k));

				unsigned int baseIndex = (k * width * height + j * width + i) * 4;

				vals[baseIndex] = (unsigned char)(noiseRed * 255.0f);
				vals[baseIndex + 1] = (unsigned char)(noiseGreen * 255.0f);
				vals[baseIndex + 2] = (unsigned char)(noiseBlue * 255.0f);
				vals[baseIndex + 3] = (unsigned char)(noiseAlpha * 255.0f);
			}
}

void HIGHOMEGA::MATH::NOISE::NoiseGenerator::RemovePast()
{
	if (vals) delete vals;

	vals = nullptr;
}

HIGHOMEGA::MATH::NOISE::NoiseGenerator::NoiseGenerator(Noise & redNoise, Noise & greenNoise, Noise & blueNoise, Noise & alphaNoise, std::string noiseName)
{
	NoiseChannels[0] = &redNoise;
	NoiseChannels[1] = &greenNoise;
	NoiseChannels[2] = &blueNoise;
	NoiseChannels[3] = &alphaNoise;

	if (NoiseChannels[0]->width != NoiseChannels[1]->width || NoiseChannels[1]->width != NoiseChannels[2]->width || NoiseChannels[2]->width != NoiseChannels[3]->width ||
		NoiseChannels[0]->height != NoiseChannels[1]->height || NoiseChannels[1]->height != NoiseChannels[2]->height || NoiseChannels[2]->height != NoiseChannels[3]->height ||
		NoiseChannels[0]->depth != NoiseChannels[1]->depth || NoiseChannels[1]->depth != NoiseChannels[2]->depth || NoiseChannels[2]->depth != NoiseChannels[3]->depth)
	{
		throw std::runtime_error("Resolution mismatch on one or more of the channels");
	}

	width = NoiseChannels[0]->width;
	height = NoiseChannels[0]->height;
	depth = NoiseChannels[0]->depth;

	unsigned int loadedDataSize;
	ResourceLoader::LOAD_LOCATION loadLocation;
	if (ResourceLoader::Load("", noiseName, &vals, loadedDataSize, loadLocation) == ResourceLoader::RESOURCE_LOAD_RESULT::RESOURCE_LOAD_SUCCESS)
	{
		if (loadedDataSize != (width * height * depth) * 4)
		{
			delete vals;
			vals = nullptr;
			throw std::runtime_error("Error loading noise from file");
		}
	}
	else
	{
		vals = new unsigned char[(width * height * depth) * 4];

		std::thread t1(FillNoiseThread, width, height, depth, NoiseChannels, vals, 0, 8);
		std::thread t2(FillNoiseThread, width, height, depth, NoiseChannels, vals, 1, 8);
		std::thread t3(FillNoiseThread, width, height, depth, NoiseChannels, vals, 2, 8);
		std::thread t4(FillNoiseThread, width, height, depth, NoiseChannels, vals, 3, 8);
		std::thread t5(FillNoiseThread, width, height, depth, NoiseChannels, vals, 4, 8);
		std::thread t6(FillNoiseThread, width, height, depth, NoiseChannels, vals, 5, 8);
		std::thread t7(FillNoiseThread, width, height, depth, NoiseChannels, vals, 6, 8);
		std::thread t8(FillNoiseThread, width, height, depth, NoiseChannels, vals, 7, 8);

		t1.join();
		t2.join();
		t3.join();
		t4.join();
		t5.join();
		t6.join();
		t7.join();
		t8.join();

		ResourceSaver::SaveBlob(noiseName, vals, (width * height * depth) * 4);
	}
}

HIGHOMEGA::MATH::NOISE::NoiseGenerator::~NoiseGenerator()
{
	RemovePast();
}

HIGHOMEGA::MATH::NOISE::PerlinWorley::PerlinWorley(PerlinNoise *inpPn, WorleyNoise *inpWn) : Noise (inpPn->width, inpPn->height, inpPn->depth)
{
	if (inpWn == nullptr) throw std::runtime_error("Worley noise is empty");

	if (inpWn->width != inpPn->width || inpWn->height != inpPn->height || inpWn->depth != inpPn->depth) throw std::runtime_error("Resolution mismatch");

	pn = inpPn;
	wn = inpWn;
}

void HIGHOMEGA::MATH::NOISE::PerlinWorley::RemovePast()
{
}

float HIGHOMEGA::MATH::NOISE::PerlinWorley::evalNoise(vec3 inpCoord)
{
	return remap (pn->evalNoise(inpCoord), wn->evalNoise(inpCoord), 1.0f, 0.0f, 1.0f);
}

inline float HIGHOMEGA::MATH::NOISE::remap(float inpVal, float low1, float high1, float low2, float high2)
{
	float frac1 = (inpVal - low1) / (high1 - low1);
	float frac1clamp = max(min(frac1, 1.0f), 0.0f);
	return low2 + frac1clamp * (high2 - low2);
}
