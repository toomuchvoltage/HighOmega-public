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

#pragma once

#include <sdl/SDL.h>
#include <util.h>
#include "vmath.h"
#include <unordered_map>

#define HIGHOMEGA_MOUSE_BUTTON_MIDDLE 517
#define HIGHOMEGA_MOUSE_BUTTON_LEFT 518
#define HIGHOMEGA_MOUSE_BUTTON_RIGHT 519

using namespace HIGHOMEGA::MATH;

namespace HIGHOMEGA
{
	namespace EVENTS
	{
		enum CMD_LIST
		{
			CMD_MOVE_FORWARD,
			CMD_MOVE_BACKWARD,
			CMD_MOVE_LEFT,
			CMD_MOVE_RIGHT,
			CMD_JUMP,
			CMD_CROUCH,
			CMD_FIRE,
			CMD_ALTFIRE,
			CMD_QUICKSAVE,
			CMD_QUICKLOAD,
			CMD_QUICKWALK,
			CMD_WRIST_PEEK,
			CMD_USE,
			CMD_FLASHLIGHT,
			CMD_MAIN_MENU,
			CMD_MEMREPORT,
			CMD_TABLET,
			CMD_HAMMER,
			CMD_AK47,
			CMD_RELOAD,
			CMD_SWITCH_TO_PT_MODE,
			CMD_LIST_SIZE,
		};
		static std::unordered_map<std::string, CMD_LIST> cmdNameToEnum = {
			{"CMD_MOVE_FORWARD", CMD_MOVE_FORWARD},
			{"CMD_MOVE_BACKWARD", CMD_MOVE_BACKWARD},
			{"CMD_MOVE_LEFT", CMD_MOVE_LEFT},
			{"CMD_MOVE_RIGHT", CMD_MOVE_RIGHT},
			{"CMD_JUMP", CMD_JUMP},
			{"CMD_CROUCH", CMD_CROUCH},
			{"CMD_FIRE", CMD_FIRE},
			{"CMD_ALTFIRE", CMD_ALTFIRE},
			{"CMD_QUICKSAVE", CMD_QUICKSAVE},
			{"CMD_QUICKLOAD", CMD_QUICKLOAD},
			{"CMD_QUICKWALK", CMD_QUICKWALK},
			{"CMD_WRIST_PEEK", CMD_WRIST_PEEK},
			{"CMD_USE", CMD_USE},
			{"CMD_FLASHLIGHT", CMD_FLASHLIGHT},
			{"CMD_MAIN_MENU", CMD_MAIN_MENU},
			{"CMD_MEMREPORT", CMD_MEMREPORT},
			{"CMD_TABLET", CMD_TABLET},
			{"CMD_HAMMER", CMD_HAMMER},
			{"CMD_AK47", CMD_AK47},
			{"CMD_RELOAD", CMD_RELOAD},
			{"CMD_SWITCH_TO_PT_MODE", CMD_SWITCH_TO_PT_MODE}
		};
		enum KEY_STATE
		{
			UP,
			DOWN
		};
		struct KEY_AND_STATE
		{
			int key = -1;
			KEY_STATE state = UP;
		};
		extern bool anyKeyDown, anyButtonDown;
		extern std::unordered_map <CMD_LIST, KEY_AND_STATE> CommandMap;
		extern bool leftMouseDown, midMouseDown, rightMouseDown;
		extern std::unordered_map <std::string, int> pressedInputConsumers;
		extern std::unordered_map <std::string, vec2> mouseMovementConsumers;
		extern std::unordered_map <std::string, float> mouseWheelConsumers;
		void TryPopulateDefaultKeybinds();
		int SDLCALL Handler();
		extern bool windowMinimized;
		bool GetStateOfAction(CMD_LIST inpCmd);
		bool ActionHasKey(CMD_LIST inpCmd);
		CMD_LIST ActionFromString(const std::string& inpCmd);
		std::string GetKeynameForAction(CMD_LIST inpCmd);
		std::string StringVersion(CMD_LIST inpCmd);
		std::string GetDisplayFriendlyName(CMD_LIST inpCmd);
	}
}