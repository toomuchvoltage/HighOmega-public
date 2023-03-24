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

#ifndef HIGHOMEGA_NOISE_H
#define HIGHOMEGA_NOISE_H

#include <vmath.h>
#include <stdlib.h>
#include <iostream>
#include <util.h>
#include <thread>

using namespace HIGHOMEGA::MATH;

namespace HIGHOMEGA
{
	namespace MATH
	{
		namespace NOISE
		{
			float remap(float inpVal, float low1, float high1, float low2, float high2);
			class Noise
			{
			public:
				virtual void RemovePast() = 0;
				virtual float evalNoise(vec3 inpCoord) = 0;
				unsigned int width, height, depth;

				Noise(unsigned int w, unsigned int h, unsigned int d);
			};
			class PerlinNoise : public Noise
			{
			private:
				float *PerlinGrad;
				vec3 getGrad(int inX, int inY, int inZ);
				void putGrad(int inX, int inY, int inZ, vec3 inpGrad);
				float evalPerlinNoise(vec3 inpCoord);
				vec3 baseSt;
				unsigned int numOctaves;

			public:
				void RemovePast();
				PerlinNoise(unsigned int w, unsigned int h, unsigned int d, vec3 inpBaseSt, unsigned int inpNumOctaves);
				~PerlinNoise();
				float evalNoise(vec3 inpCoord);
			};
			class WorleyNoise : public Noise
			{
			private:
				vec3 *bins;
				int binW, binH, binD, binDim;
				bool reverse;
				unsigned int numOctaves;
				float evalNoiseWorley(vec3 inpCoord);

			public:
				void RemovePast();
				WorleyNoise(unsigned int w, unsigned int h, unsigned int d, unsigned int inpBinDim, bool inpReverse, unsigned int inpNumOctaves = 0u);
				~WorleyNoise();
				float evalNoise(vec3 inpCoord);
			};
			class PerlinWorley : public Noise
			{
			private:
				PerlinNoise *pn;
				WorleyNoise *wn;

			public:
				PerlinWorley(PerlinNoise *inpPn, WorleyNoise *inpWn);
				void RemovePast();
				float evalNoise(vec3 inpCoord);
			};
			class NoiseGenerator
			{
			private:
				Noise *NoiseChannels[4];
				static void FillNoiseThread(unsigned int width, unsigned int height, unsigned int depth, Noise **allNoisesArray, unsigned char *vals, int threadNum, int maxThreads);

			public:
				unsigned int width, height, depth;
				unsigned char *vals;

				void RemovePast();
				NoiseGenerator(Noise & redNoise, Noise & greenNoise, Noise & blueNoise, Noise & alphaNoise, std::string noiseName);
				~NoiseGenerator();
			};
		}
	}
}

#endif
