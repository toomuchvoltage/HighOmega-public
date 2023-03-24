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

#ifndef HIGHOMEGA_UTIL_H
#define HIGHOMEGA_UTIL_H

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <time.h>
#include "common.h"
#include <process.h>
#include <vector>
#include <vmath.h>
#include <shared_mutex>
#include <chrono>
#include <random>
#include <functional>

using namespace HIGHOMEGA::MATH;

namespace HIGHOMEGA
{
	extern std::shared_mutex CommonSharedMutex;
	extern std::mt19937_64 mersenneTwister64BitPRNG;
	class ResourceLoader
	{
	public:
		enum FILE_LOAD_RESULT
		{
			FILE_LOAD_SUCCESS = 0,
			FILE_NOT_FOUND,
			FILE_EMPTY
		};
		enum LOAD_CHOSEN_ASSET
		{
			FILE = 0,
			IMG_KTX
		};
		enum LOAD_LOCATION
		{
			ORIGINAL = 0,
			COMMON_FOLDER
		};
		enum RESOURCE_LOAD_RESULT
		{
			RESOURCE_LOAD_SUCCESS = 0,
			RESOURCE_NOT_FOUND,
			RESOURCE_EMPTY
		};
		static std::string GetFileName(std::string filePath);
		static FILE_LOAD_RESULT LoadFile(std::string fileName, unsigned char ** content, unsigned int & size);
		static bool FindAsset(std::string belong, std::string fileName, std::string & finalPath, LOAD_LOCATION & loadLocation, LOAD_CHOSEN_ASSET & loadChosenAsset, bool searchingCommonFolder = false);
		static RESOURCE_LOAD_RESULT Load(std::string belong, std::string fileName, unsigned char **content, unsigned int & size, LOAD_LOCATION & loadLocation, LOAD_CHOSEN_ASSET & loadChosenAsset);
		static bool DirectoryExists(std::string dirName);
		enum TGA_PARSE_RESULT
		{
			TGA_SUCCESS = 0,
			BAD_RLE,
			UNSUPPORTED_BPP
		};
		static TGA_PARSE_RESULT TGALoad(unsigned char * inpData, int *w, int *h, unsigned char **data, int *outBpp);
		static void WAVLoad(unsigned char *inpData, short int & numChannels, short int & numBits, void **audioData, unsigned int & audioSize, unsigned int & freq);
		static void RGBtoRGBA(int w, int h, unsigned char **data);
		static void RtoRGBA(int w, int h, unsigned char **data);
	};
	class ResourceSaver
	{
	public:
		static void SavePPM(std::string fileName, unsigned char *pixData, unsigned int width, unsigned int height);
		static void SaveBlob(std::string fileName, unsigned char *data, unsigned int dataSize);
	};
	template <typename T>
	std::string toStringPrecision(const T a_value, const int n = 6);
	class LOG
	{
	public:
		std::stringstream stream;

		LOG();
		~LOG();
	};
	template <typename T>
	LOG& operator<<(LOG& record, T&& t) {
		record.stream << std::forward<T>(t);
		return record;
	}
	template <typename T>
	LOG& operator<<(LOG&& record, T&& t) {
		return record << std::forward<T>(t);
	}
	void GetCurrentTimeAndDate(char *ret_str, int ret_size);
	class TimerObject
	{
	private:
		std::chrono::time_point<std::chrono::high_resolution_clock> t1;
	public:
		void Start();
		double Diff();
	};
	double GetSystemTimeMsSinceEpoch();
	void removeAll(std::string& s, std::string& p);
	template<typename T>
	inline std::string toStringPrecision(const T a_value, const int n)
	{
		std::ostringstream out;
		out.precision(n);
		out << std::fixed << a_value;
		return out.str();
	}
	namespace INSTRUMENTATION
	{
		class Instrument
		{
		private:
			static bool enableInstruments;
			float minTime = 10000000.0f, maxTime = -10000000.0f, totalTime = 0.0f;
			unsigned int countTakes = 0;
			TimerObject timer;

		public:
			static void EnableGlobally();
			static void DisableGlobally();
			void Start();
			void End();
			std::string Results(unsigned int maxSize = UINT_MAX);
			std::string ResultsMs(unsigned int maxSize = UINT_MAX);
		};
		class StatsCollector
		{
		private:
			unsigned long long minTime, maxTime, totalTime = 0ull;
			unsigned int countTakes = 0;
			bool firstTime = true;

		public:
			void Feed(unsigned long long val);
			std::string Results(unsigned int maxSize = UINT_MAX);
			std::string ResultsMs(unsigned int maxSize = UINT_MAX);
		};
		class FPSCounter
		{
		private:
			static float timeMeasure;
			static unsigned int countFPS;
			static unsigned int lastFPS;
			static bool newReport;
			static TimerObject timer;

		public:
			static void Start();
			static void End();
			static unsigned int Report();
			static void Report(std::function<void(unsigned int)> callAtReport);
		};
	}
};
#endif