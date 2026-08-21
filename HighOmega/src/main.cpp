/*
	Copyright (c) 2026 TooMuchVoltage Software Inc.

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

#include <stdio.h>
#include <math.h>
#include "world.h"
#include <iostream>

extern "C" {
	__declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
	__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}

using namespace HIGHOMEGA;
using namespace HIGHOMEGA::WORLD;
extern HIGHOMEGA::RENDER::FrustumClass HIGHOMEGA::RENDER::MainFrustum;
extern HIGHOMEGA::RENDER::ScreenSizeClass HIGHOMEGA::RENDER::ScreenSize;

int main(int argc, char *argv[])
{
	std::string wrongCmdLine;
	int nCmdParams;
	wrongCmdLine = "Wrong params. Command line usage is: HighOmega [hwrt|sdfbvh] [1920x1080|1280x720|1024x768|800x600|640x480] [windowed|windowed_fullscreen|fullscreen]\n";
	nCmdParams = 4;
	try {
		bool cmdOptHwRt = true;
		unsigned int cmdOptFullRes = 4, cmdOptFullResW = 1920, cmdOptFullResH = 1080;
		WINDOW_MODE cmdOptWindowedMode = WINDOW_MODE::WINDOWED_FULLSCREEN;
		ConfigINI config;
		if (config.Load("config.ini"))
		{
			std::string cmdOptFullResStr;
			int cmdOptWindowedModeInt, cmdOptHwRtInt;
			if (config.Eval("[video]", "fullResSelection", cmdOptFullResStr))
			{
				if (cmdOptFullResStr == "1920x1080") { cmdOptFullRes = 4; cmdOptFullResW = 1920; cmdOptFullResH = 1080; }
				else if (cmdOptFullResStr == "1280x720") { cmdOptFullRes = 3; cmdOptFullResW = 1280; cmdOptFullResH = 720; }
				else if (cmdOptFullResStr == "1024x768") { cmdOptFullRes = 2; cmdOptFullResW = 1024; cmdOptFullResH = 768; }
				else if (cmdOptFullResStr == "800x600") { cmdOptFullRes = 1; cmdOptFullResW = 800; cmdOptFullResH = 600; }
				else if (cmdOptFullResStr == "640x480") { cmdOptFullRes = 0; cmdOptFullResW = 640; cmdOptFullResH = 480; }
			}
			if (config.Eval("[video]", "windowedSelection", cmdOptWindowedModeInt)) cmdOptWindowedMode = (WINDOW_MODE)cmdOptWindowedModeInt;
			if (config.Eval("[video]", "hwrtSelection", cmdOptHwRtInt)) cmdOptHwRt = (bool)cmdOptHwRtInt;
			config.ExtractKeybBindings();
			float globalGainFetch, musicGainFetch, sfxGainFetch;
			if (config.Eval("[audio]", "globalGain", globalGainFetch)) AudioSystem.SetSFXGain(globalGainFetch);
			if (config.Eval("[audio]", "musicGainFetch", musicGainFetch)) AudioSystem.SetSFXGain(musicGainFetch);
			if (config.Eval("[audio]", "sfxGainFetch", sfxGainFetch)) AudioSystem.SetSFXGain(sfxGainFetch);
		}
		if (argc > 1)
		{
			if (argc != nCmdParams) FATAL_ERROR(wrongCmdLine.c_str());

			std::string technique, screenRes, coarseness, trigridcoarseness, windowed;

			technique = std::string(argv[1]);
			screenRes = std::string(argv[2]);
			windowed = std::string(argv[3]);

			if (technique == "hwrt") cmdOptHwRt = true;
			else if (technique == "sdfbvh") cmdOptHwRt = false;
			else FATAL_ERROR(wrongCmdLine.c_str());

			if (screenRes == "1920x1080") { cmdOptFullRes = 4; cmdOptFullResW = 1920; cmdOptFullResH = 1080; }
			else if (screenRes == "1280x720") { cmdOptFullRes = 3; cmdOptFullResW = 1280; cmdOptFullResH = 720; }
			else if (screenRes == "1024x768") { cmdOptFullRes = 2; cmdOptFullResW = 1024; cmdOptFullResH = 768; }
			else if (screenRes == "800x600") { cmdOptFullRes = 1; cmdOptFullResW = 800; cmdOptFullResH = 600; }
			else if (screenRes == "640x480") { cmdOptFullRes = 0; cmdOptFullResW = 640; cmdOptFullResH = 480; }
			else FATAL_ERROR(wrongCmdLine.c_str());

			if (windowed == "windowed") cmdOptWindowedMode = WINDOW_MODE::WINDOWED;
			else if (windowed == "windowed_fullscreen") cmdOptWindowedMode = WINDOW_MODE::WINDOWED_FULLSCREEN;
			else if (windowed == "fullscreen") cmdOptWindowedMode = WINDOW_MODE::FULLSCREEN;
			else FATAL_ERROR(wrongCmdLine.c_str());
		}

		ScreenSize.Create(cmdOptFullResW, cmdOptFullResH);
		MainFrustum.CreatePerspective(vec3(0.0f, 0.0f, 1.0f), vec3(0.0f, 0.0f, -1.0f), vec3(0.0f, 1.0f, 0.0f), 65.0f, (float)ScreenSize.width / (float)ScreenSize.height, 0.1f, 10000.0f);
		InitGraphicsSubSystem(cmdOptHwRt, cmdOptWindowedMode);
		if (InstanceClass::LowMemoryDevice())
		{
			FATAL_ERROR("VRAM too low. This demo requires at least 3.7GBs of reported video ram.");
		}
		
		PipelineSetupReturn pipelineSetupReturn;
		pipelineSetupReturn.newMap = "zones";
		pipelineSetupReturn.newMapBelong = "assets/maps/journey/"; // Switch to "source_material/dev_test_maps/test_zones/" for the engine test arena

		// For the main demo map, we're turning this on which will change some parameters in the engine for the sake of fidelity
		HIGHOMEGA::creativeLicense = (pipelineSetupReturn.newMapBelong == "assets/maps/journey/");

		for(;;)
		{
			DefaultPipelineSetupClass *gamePipeline = new DefaultPipelineSetupClass(pipelineSetupReturn.newMap, pipelineSetupReturn.newMapBelong, cmdOptHwRt, cmdOptFullRes, cmdOptWindowedMode);
			pipelineSetupReturn = gamePipeline->Run();
			if (gamePipeline->IsApplicationQuitting()) { delete gamePipeline; break; }
			delete gamePipeline;
		}

	} catch (const std::runtime_error&) {
		return -1;
	} catch (const std::bad_alloc& e) {
		std::string outError = "Allocation failure: ";
		outError += e.what();
		LOG() << outError;
		return -1;
	}
	return 0;
}