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

#ifndef HIGHOMEGA_INPUT_H
#define HIGHOMEGA_INPUT_H

#include <sdl/SDL.h>
#include <util.h>
#include "vmath.h"
#include <unordered_map>

#define HIGHOMEGA_MAXIMUM_SDLK 520
#define HIGHOMEGA_MOUSE_BUTTON_LEFT 518
#define HIGHOMEGA_MOUSE_BUTTON_RIGHT 519

using namespace HIGHOMEGA::MATH;

namespace HIGHOMEGA
{
	namespace EVENTS
	{
		enum CMD_LIST
		{
			CMD_NO_ACTION,
			CMD_MOVE_FORWARD,
			CMD_MOVE_BACKWARD,
			CMD_MOVE_LEFT,
			CMD_MOVE_RIGHT,
			CMD_JUMP,
			CMD_COLORGRADING,
			CMD_CROUCH,
			CMD_FIRE,
			CMD_ALTFIRE,
			CMD_EXITGAME,
			CMD_QUICKSAVE,
			CMD_USE,
			CMD_DROPOBJECT,
			CMD_WEAPON1,
			CMD_WEAPON2,
			CMD_FLASHLIGHT,
			CMD_MAIN_MENU,
			CMD_HELP,
			CMD_QTIP,
			CMD_PAUSE,
			CMD_SWITCH_TO_PT_MODE,
			CMD_LIST_SIZE,
		};
		enum KEY_STATE
		{
			UP,
			DOWN
		};
		extern unsigned int KeyStates[2][HIGHOMEGA_MAXIMUM_SDLK];
		extern bool leftMouseDown, midMouseDown, rightMouseDown;
		extern std::unordered_map <std::string,vec2> mouseMovementConsumers;
		int SDLCALL Handler();
		extern bool windowMinimized;
		bool GetStateOfKeybAction(CMD_LIST inpCmd);
		bool GetScanCodeForAction(CMD_LIST inpCmd, SDL_Scancode & outScanCode);
	}
}

#endif