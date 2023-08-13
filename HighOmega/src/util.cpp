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

#include <util.h>

#pragma warning(disable:4996)

std::shared_mutex HIGHOMEGA::CommonSharedMutex;
std::mt19937_64 HIGHOMEGA::mersenneTwister64BitPRNG(std::random_device{}());

bool HIGHOMEGA::INSTRUMENTATION::Instrument::enableInstruments = false; // We skip the first frame cause we're recording command buffers and PSOs then...

HIGHOMEGA::INSTRUMENTATION::Instrument updateInstrument;
HIGHOMEGA::INSTRUMENTATION::Instrument traceInstrument;
HIGHOMEGA::INSTRUMENTATION::Instrument frameInstrument;

float HIGHOMEGA::INSTRUMENTATION::FPSCounter::timeMeasure = 0.0f;
unsigned int HIGHOMEGA::INSTRUMENTATION::FPSCounter::lastFPS = 0;
unsigned int HIGHOMEGA::INSTRUMENTATION::FPSCounter::countFPS = 0;
bool HIGHOMEGA::INSTRUMENTATION::FPSCounter::newReport = false;
HIGHOMEGA::TimerObject HIGHOMEGA::INSTRUMENTATION::FPSCounter::timer;

std::string HIGHOMEGA::ResourceLoader::GetFileName(std::string filePath)
{
	std::size_t sepPos = filePath.rfind('/');

	if (sepPos != std::string::npos)
	{
		return filePath.substr(sepPos + 1, filePath.size() - 1);
	}
	return "";
}

HIGHOMEGA::ResourceLoader::FILE_LOAD_RESULT HIGHOMEGA::ResourceLoader::LoadFile(std::string fileName, unsigned char ** content, unsigned int & size)
{
	std::ifstream file(fileName.c_str(), std::ios::in | std::ios::ate | std::ios::binary);
	if (file.is_open())
	{
		size = (unsigned int)file.tellg();
		if (size == 0) return FILE_EMPTY;
		*content = new unsigned char[size];
		file.seekg(0, std::ios::beg);
		file.read((char *)*content, size);
		file.close();
		return FILE_LOAD_SUCCESS;
	}
	else
		return FILE_NOT_FOUND;
}

bool HIGHOMEGA::ResourceLoader::FindAsset(std::string belong, std::string fileName, std::string & finalPath, LOAD_LOCATION & loadLocation, bool searchingCommonFolder)
{
	std::string fullPath;

	fullPath = belong + fileName;
	std::ifstream file(fullPath.c_str(), std::ios::in | std::ios::ate | std::ios::binary);
	if (file.is_open() && file.tellg() > 0)
	{
		file.close();
		loadLocation = searchingCommonFolder ? COMMON_FOLDER : ORIGINAL;
		finalPath = fullPath;
		return true;
	}
	if (file.is_open()) file.close();

	if (!searchingCommonFolder) return FindAsset("assets/common/", fileName, finalPath, loadLocation, true);

	return false;
}

HIGHOMEGA::ResourceLoader::RESOURCE_LOAD_RESULT HIGHOMEGA::ResourceLoader::Load(std::string belong, std::string fileName, unsigned char **content, unsigned int & size, LOAD_LOCATION & loadLocation)
{
	std::string finalPath;
	if (FindAsset(belong, fileName, finalPath, loadLocation))
	{
		if (LoadFile(finalPath, content, size) == FILE_EMPTY)
			return RESOURCE_EMPTY;
		else
			return RESOURCE_LOAD_SUCCESS;
	}
	else
		return RESOURCE_NOT_FOUND;
}

bool HIGHOMEGA::ResourceLoader::DirectoryExists(std::string dirName)
{
	return std::filesystem::is_directory(dirName);
}

HIGHOMEGA::ResourceLoader::TGA_PARSE_RESULT HIGHOMEGA::ResourceLoader::TGALoad(unsigned char * inpData, int *w, int *h, unsigned char **data, int *outBpp)
{
	unsigned int DATA_OFFSET = 0;
	char idLength = inpData[DATA_OFFSET++];

	DATA_OFFSET++;

	bool RLE;
	int rleRead = inpData[DATA_OFFSET++];
	if (rleRead == 2 || rleRead == 3) RLE = false;
	else if (rleRead == 10 || rleRead == 11) RLE = true;
	else return BAD_RLE;

	bool grayScale = false;
	if (rleRead == 3 || rleRead == 11)
		grayScale = true;

	DATA_OFFSET += 9;

	unsigned int width, height;

	width = (int)inpData[DATA_OFFSET++];
	width += (int)inpData[DATA_OFFSET++] << 8;

	height = (int)inpData[DATA_OFFSET++];
	height += (int)inpData[DATA_OFFSET++] << 8;

	int bpp = (int)inpData[DATA_OFFSET++];
	if (grayScale) bpp = 8;
	*outBpp = bpp / 8;
	if (!grayScale && bpp != 32 && bpp != 24) return UNSUPPORTED_BPP;

	int attrib_byte = (int)inpData[DATA_OFFSET++];

	DATA_OFFSET += (int)idLength;

	int Bpp = bpp / 8;
	*data = new unsigned char[width*height*Bpp*sizeof(unsigned char)];

	if (RLE == false)
	{
		for (int i = 0; i != height; i++)
			for (int j = 0; j != width; j++)
			{
				int pixel_index = (i*width*Bpp) + (j*Bpp);
				if (bpp == 32)
				{
					(*data)[pixel_index + 2] = inpData[DATA_OFFSET++];
					(*data)[pixel_index + 1] = inpData[DATA_OFFSET++];
					(*data)[pixel_index] = inpData[DATA_OFFSET++];
					(*data)[pixel_index + 3] = inpData[DATA_OFFSET++];
				}
				else if (bpp == 24)
				{
					(*data)[pixel_index + 2] = inpData[DATA_OFFSET++];
					(*data)[pixel_index + 1] = inpData[DATA_OFFSET++];
					(*data)[pixel_index] = inpData[DATA_OFFSET++];
				}
				else
				{
					(*data)[pixel_index] = inpData[DATA_OFFSET++];
				}
			}
	}
	else
	{
		int offset = 0;
		while (offset != width*height*Bpp)
		{
			unsigned char pkt = inpData[DATA_OFFSET++];
			if ((pkt & 0x80) != 0) // RLE packet
			{
				int size = (int)((pkt & 0x7F) + 1);
				if (bpp == 32)
				{
					unsigned char b = inpData[DATA_OFFSET++];
					unsigned char g = inpData[DATA_OFFSET++];
					unsigned char r = inpData[DATA_OFFSET++];
					unsigned char a = inpData[DATA_OFFSET++];
					for (int i = 0; i != size; i++)
					{
						(*data)[offset + 2] = b;
						(*data)[offset + 1] = g;
						(*data)[offset] = r;
						(*data)[offset + 3] = a;
						offset += 4;
					}
				}
				else if (bpp == 24)
				{
					unsigned char b = inpData[DATA_OFFSET++];
					unsigned char g = inpData[DATA_OFFSET++];
					unsigned char r = inpData[DATA_OFFSET++];
					for (int i = 0; i != size; i++)
					{
						(*data)[offset + 2] = b;
						(*data)[offset + 1] = g;
						(*data)[offset] = r;
						offset += 3;
					}
				}
				else
				{
					unsigned char gray = inpData[DATA_OFFSET++];
					for (int i = 0; i != size; i++)
					{
						(*data)[offset] = gray;
						offset++;
					}
				}
			}
			else // RAW packet
			{
				int size = (int)((pkt & 0x7F) + 1);
				for (int i = 0; i != size; i++)
				{
					if (bpp == 32)
					{
						int base = offset + (i * 4);
						(*data)[base + 2] = inpData[DATA_OFFSET++];
						(*data)[base + 1] = inpData[DATA_OFFSET++];
						(*data)[base] = inpData[DATA_OFFSET++];
						(*data)[base + 3] = inpData[DATA_OFFSET++];
					}
					else if (bpp == 24)
					{
						int base = offset + (i * 3);
						(*data)[base + 2] = inpData[DATA_OFFSET++];
						(*data)[base + 1] = inpData[DATA_OFFSET++];
						(*data)[base] = inpData[DATA_OFFSET++];
					}
					else
					{
						int base = offset + i;
						(*data)[base] = inpData[DATA_OFFSET++];
					}
				}
				if (bpp == 32)
					offset += size * 4;
				else if (bpp == 24)
					offset += size * 3;
				else
					offset += size;
			}
		}
	}

	*w = width;
	*h = height;

	return TGA_SUCCESS;
}

void HIGHOMEGA::ResourceLoader::WAVLoad(unsigned char *inpData, short int & numChannels, short int & numBits, void **audioData, unsigned int & audioSize, unsigned int & freq)
{
	unsigned int DATA_OFFSET = 22;
	numChannels = *((short int *)(&inpData[DATA_OFFSET]));
	DATA_OFFSET += sizeof(short int);
	freq = *((unsigned int *)(&inpData[DATA_OFFSET]));
	DATA_OFFSET += sizeof(unsigned int) + 6;
	numBits = *((short int *)(&inpData[DATA_OFFSET]));

	unsigned int audioChunkOffset = 0;
	for (;; audioChunkOffset++) {
		if (inpData[audioChunkOffset] == 'd' && inpData[audioChunkOffset + 1] == 'a' && inpData[audioChunkOffset + 2] == 't' && inpData[audioChunkOffset + 3] == 'a') break;
	}
	DATA_OFFSET = audioChunkOffset + sizeof(unsigned int);
	audioSize = *((unsigned int *)(&inpData[DATA_OFFSET]));
	DATA_OFFSET += sizeof (unsigned int);
	*audioData = new unsigned char[audioSize];
	memcpy(*audioData, (void *)&inpData[DATA_OFFSET], audioSize);
}

void HIGHOMEGA::ResourceLoader::RGBtoRGBA(int w, int h, unsigned char **data)
{
	unsigned char *out_data;
	out_data = new unsigned char[w * h * 4];
	for (int i = 0; i != w; i++)
		for (int j = 0; j != h; j++)
		{
			int byte_index = (j * w + i)*4;
			int input_index = (j * w + i)*3;
			out_data[byte_index] = (*data)[input_index];
			out_data[byte_index+1] = (*data)[input_index+1];
			out_data[byte_index+2] = (*data)[input_index+2];
			out_data[byte_index+3] = (unsigned char)255;
		}

	delete[] (*data);
	*data = out_data;
}

void HIGHOMEGA::ResourceLoader::RtoRGBA(int w, int h, unsigned char ** data)
{
	unsigned char *out_data;
	out_data = new unsigned char[w * h * 4];
	for (int i = 0; i != w; i++)
		for (int j = 0; j != h; j++)
		{
			int byte_index = (j * w + i) * 4;
			int input_index = (j * w + i);
			unsigned char grayVal = (*data)[input_index];
			out_data[byte_index] = grayVal;
			out_data[byte_index + 1] = grayVal;
			out_data[byte_index + 2] = grayVal;
			out_data[byte_index + 3] = (unsigned char)255;
		}

	delete[](*data);
	*data = out_data;
}

void HIGHOMEGA::GetCurrentTimeAndDate(char *ret_str,int ret_size)
{
	time_t rawtime = {};
	struct tm timeinfo = {};

	time(&rawtime);
	localtime_s(&timeinfo,&rawtime);
	asctime_s(ret_str,ret_size,&timeinfo);
}

double HIGHOMEGA::GetSystemTimeMsSinceEpoch()
{
	return (double)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void HIGHOMEGA::removeAll(std::string & s, std::string & p)
{
	std::string::size_type n = p.length();
	for (std::string::size_type i = s.find(p); i != std::string::npos; i = s.find(p)) {
		s.erase(i, n);
	}
}

void HIGHOMEGA::TimerObject::Start()
{
	t1 = std::chrono::high_resolution_clock::now();
}

double HIGHOMEGA::TimerObject::Diff()
{
	std::chrono::duration<double> diff = std::chrono::high_resolution_clock::now() - t1;
	return diff.count();
}

void HIGHOMEGA::ResourceSaver::SavePPM(std::string fileName, unsigned char *pixData, unsigned int width, unsigned int height)
{
	std::ofstream saveStream;

	saveStream.open(fileName);

	saveStream << "P3\n";
	saveStream << width << " " << height << "\n";
	saveStream << "255\n";
	for (int j = 0; j != height; j++)
	{
		for (int i = 0; i != width; i++)
		{
			unsigned int curIndex = ((j * width) + i) * 4;
			unsigned int redVal = (unsigned int)pixData[curIndex];
			unsigned int greenVal = (unsigned int)pixData[curIndex + 1];
			unsigned int blueVal = (unsigned int)pixData[curIndex + 2];
			saveStream << " " << std::to_string (redVal) << " " << std::to_string (greenVal) << " " << std::to_string (blueVal) << " \t";
		}
		saveStream << "\n";
	}

	saveStream.close();
}

void HIGHOMEGA::ResourceSaver::SaveBlob(std::string fileName, unsigned char * data, unsigned int dataSize)
{
	std::ofstream saveStream;

	saveStream.open(fileName, std::ios::out | std::ios::binary);

	saveStream.write((const char *)data, dataSize);

	saveStream.close();
}

HIGHOMEGA::LOG::LOG()
{
}

HIGHOMEGA::LOG::~LOG()
{
	static bool firstTime = true;
	static std::mutex logMutex;

	logMutex.lock();
	std::ofstream outfile;
	int mode = std::ofstream::out;
	if (firstTime == false) mode |= std::ofstream::app;
	unsigned int curTimeSec = (unsigned int)(std::time(nullptr) / 3600);
	outfile.open(std::string("highomega_log") + std::to_string(curTimeSec) + ".txt", mode);
	char cur_time[256];
	GetCurrentTimeAndDate(cur_time, 256);
	cur_time[strlen(cur_time) - 1] = '\0';
	std::string logTime = "[ ";
	logTime += cur_time;
	logTime += " ] ";
	if (firstTime)
	{
		std::string logTag = logTime;
		logTag += "HighOmega ";
		logTag += std::to_string(HIGHOMEGA_VERSION);
		logTag += " Started.\n";
		outfile << logTag;
		std::cout << logTag;
	}
	std::string logOutput = logTime;
	logOutput += stream.str();
	logOutput += "\n";
	outfile << logOutput;
	outfile.close();
	logMutex.unlock();

	std::cout << logOutput;

	firstTime = false;
}

void HIGHOMEGA::INSTRUMENTATION::Instrument::EnableGlobally()
{
	enableInstruments = true;
}

void HIGHOMEGA::INSTRUMENTATION::Instrument::DisableGlobally()
{
	enableInstruments = false;
}

void HIGHOMEGA::INSTRUMENTATION::Instrument::Start()
{
	if (!enableInstruments) return;
	timer.Start();
}

void HIGHOMEGA::INSTRUMENTATION::Instrument::End()
{
	if (!enableInstruments) return;
	float timeSpent = (float)timer.Diff();
	minTime = min(minTime, timeSpent);
	maxTime = max(maxTime, timeSpent);
	totalTime += timeSpent;
	countTakes++;
}

std::string HIGHOMEGA::INSTRUMENTATION::Instrument::Results(unsigned int maxSize)
{
	std::string retVal = "";
	if (countTakes > 0)
	{
		retVal += "min: ";
		retVal += std::to_string(minTime);
		retVal += " max: ";
		retVal += std::to_string(maxTime);
		retVal += " avg: ";
		retVal += std::to_string(totalTime / (float)countTakes);
	}
	if (maxSize == UINT_MAX)
		return retVal;
	else
		return retVal.substr(0, maxSize);
}

std::string HIGHOMEGA::INSTRUMENTATION::Instrument::ResultsMs(unsigned int maxSize)
{
	std::string retVal = "";
	if (countTakes > 0)
	{
		retVal += "min: ";
		retVal += toStringPrecision(minTime * 1000.0f, 2);
		retVal += " max: ";
		retVal += toStringPrecision(maxTime * 1000.0f, 2);
		retVal += " avg: ";
		retVal += toStringPrecision((totalTime / (float)countTakes) * 1000.0f, 2);
	}
	if (maxSize == UINT_MAX)
		return retVal;
	else
		return retVal.substr(0, maxSize);
}

void HIGHOMEGA::INSTRUMENTATION::StatsCollector::Feed(unsigned long long val)
{
	if (firstTime)
	{
		minTime = maxTime = val;
		firstTime = false;
	}
	else
	{
		minTime = min(minTime, val);
		maxTime = max(maxTime, val);
	}
	totalTime += val;
	countTakes++;
}

std::string HIGHOMEGA::INSTRUMENTATION::StatsCollector::Results(unsigned int maxSize)
{
	std::string retVal = "";
	if (countTakes > 0)
	{
		retVal += "min: ";
		retVal += std::to_string(minTime);
		retVal += " max: ";
		retVal += std::to_string(maxTime);
		retVal += " avg: ";
		retVal += std::to_string((double)totalTime / (double)countTakes);
	}
	if (maxSize == UINT_MAX)
		return retVal;
	else
		return retVal.substr(0, maxSize);
}

std::string HIGHOMEGA::INSTRUMENTATION::StatsCollector::ResultsMs(unsigned int maxSize)
{
	std::string retVal = "";
	if (countTakes > 0)
	{
		retVal += "min: ";
		retVal += toStringPrecision(minTime * 0.000001, 2);
		retVal += " max: ";
		retVal += toStringPrecision(maxTime * 0.000001, 2);
		retVal += " avg: ";
		retVal += toStringPrecision(((double)totalTime / (double)countTakes) * 0.000001, 2);
	}
	if (maxSize == UINT_MAX)
		return retVal;
	else
		return retVal.substr(0, maxSize);
}

void HIGHOMEGA::INSTRUMENTATION::FPSCounter::Start()
{
	timer.Start();
}

void HIGHOMEGA::INSTRUMENTATION::FPSCounter::End()
{
	timeMeasure += (float)timer.Diff();
	if (timeMeasure > 1.0f)
	{
		timeMeasure -= 1.0f;
		lastFPS = countFPS;
		countFPS = 1;
		newReport = true;
	}
	else
		countFPS++;
}

unsigned int HIGHOMEGA::INSTRUMENTATION::FPSCounter::Report()
{
	return lastFPS;
}

void HIGHOMEGA::INSTRUMENTATION::FPSCounter::Report(std::function<void(unsigned int)> callAtReport)
{
	if (newReport)
	{
		callAtReport(lastFPS);
		newReport = false;
	}
}
