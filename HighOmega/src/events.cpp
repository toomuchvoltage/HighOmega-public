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

#include "events.h"

unsigned int HIGHOMEGA::EVENTS::KeyStates[2][HIGHOMEGA_MAXIMUM_SDLK];
bool HIGHOMEGA::EVENTS::leftMouseDown, HIGHOMEGA::EVENTS::midMouseDown, HIGHOMEGA::EVENTS::rightMouseDown;
std::unordered_map <std::string, vec2> HIGHOMEGA::EVENTS::mouseMovementConsumers;
bool HIGHOMEGA::EVENTS::windowMinimized = false;

int SDLCALL HIGHOMEGA::EVENTS::Handler()
{
	static bool firstRun = true;

	if (firstRun)
	{
		SDL_SetRelativeMouseMode(SDL_TRUE);

		for (int i = 0; i != HIGHOMEGA_MAXIMUM_SDLK; i++)
		{
			KeyStates[0][i] = CMD_NO_ACTION;
			KeyStates[1][i] = UP;
		}
		KeyStates[0][SDL_SCANCODE_W] = CMD_MOVE_FORWARD;
		KeyStates[0][SDL_SCANCODE_S] = CMD_MOVE_BACKWARD;
		KeyStates[0][SDL_SCANCODE_A] = CMD_MOVE_LEFT;
		KeyStates[0][SDL_SCANCODE_D] = CMD_MOVE_RIGHT;
		KeyStates[0][SDL_SCANCODE_SPACE] = CMD_JUMP;
		KeyStates[0][SDL_SCANCODE_LCTRL] = CMD_CROUCH;
		KeyStates[0][HIGHOMEGA_MOUSE_BUTTON_LEFT] = CMD_FIRE;
		KeyStates[0][HIGHOMEGA_MOUSE_BUTTON_RIGHT] = CMD_ALTFIRE;
		KeyStates[0][SDL_SCANCODE_F10] = CMD_EXITGAME;
		KeyStates[0][SDL_SCANCODE_F1] = CMD_HELP;
		KeyStates[0][SDL_SCANCODE_E] = CMD_USE;
		KeyStates[0][SDL_SCANCODE_C] = CMD_COLORGRADING;
		KeyStates[0][SDL_SCANCODE_G] = CMD_DROPOBJECT;
		KeyStates[0][SDL_SCANCODE_1] = CMD_WEAPON1;
		KeyStates[0][SDL_SCANCODE_2] = CMD_WEAPON2;
		KeyStates[0][SDL_SCANCODE_L] = CMD_FLASHLIGHT;
		KeyStates[0][SDL_SCANCODE_ESCAPE] = CMD_MAIN_MENU;
		KeyStates[0][SDL_SCANCODE_Q] = CMD_QTIP;
		KeyStates[0][SDL_SCANCODE_P] = CMD_SWITCH_TO_PT_MODE;

		firstRun = false;
	}

	SDL_Event events;
	while (SDL_PollEvent(&events))
	{
		switch (events.type)
		{
		case SDL_KEYDOWN:
			KeyStates[1][events.key.keysym.scancode] = DOWN;
			break;
		case SDL_KEYUP:
			KeyStates[1][events.key.keysym.scancode] = UP;
			break;
		case SDL_MOUSEBUTTONDOWN:
			switch (events.button.button)
			{
			case SDL_BUTTON_LEFT:
				leftMouseDown = true;
				break;
			case SDL_BUTTON_MIDDLE:
				midMouseDown = true;
				break;
			case SDL_BUTTON_RIGHT:
				rightMouseDown = true;
				break;
			}
			break;
		case SDL_MOUSEBUTTONUP:
			switch (events.button.button)
			{
			case SDL_BUTTON_LEFT:
				leftMouseDown = false;
				break;
			case SDL_BUTTON_MIDDLE:
				midMouseDown = false;
				break;
			case SDL_BUTTON_RIGHT:
				rightMouseDown = false;
				break;
			}
			break;
		case SDL_MOUSEMOTION:
			for (std::pair<const std::string, vec2> & curConsumer : mouseMovementConsumers) {
				curConsumer.second += vec2((float)events.motion.xrel, -(float)events.motion.yrel);
			}
			break;
		case SDL_WINDOWEVENT:
			if (events.window.event == SDL_WINDOWEVENT_MINIMIZED)
				HIGHOMEGA::EVENTS::windowMinimized = true;
			else if (events.window.event == SDL_WINDOWEVENT_RESTORED)
				HIGHOMEGA::EVENTS::windowMinimized = false;
			break;
		}
	}

	Uint8 mouse_state = SDL_GetMouseState(nullptr, nullptr);
	if (mouse_state & SDL_BUTTON(SDL_BUTTON_LEFT)) KeyStates[1][HIGHOMEGA_MOUSE_BUTTON_LEFT] = DOWN;
	else KeyStates[1][HIGHOMEGA_MOUSE_BUTTON_LEFT] = UP;
	if (mouse_state & SDL_BUTTON(SDL_BUTTON_RIGHT)) KeyStates[1][HIGHOMEGA_MOUSE_BUTTON_RIGHT] = DOWN;
	else KeyStates[1][HIGHOMEGA_MOUSE_BUTTON_RIGHT] = UP;

	return 1;
}

bool HIGHOMEGA::EVENTS::GetStateOfKeybAction(CMD_LIST inpCmd)
{
	for (int i = 0; i != HIGHOMEGA_MAXIMUM_SDLK; i++)
		if (KeyStates[0][i] == inpCmd) return KeyStates[1][i]==DOWN;
	return false;
}

bool HIGHOMEGA::EVENTS::GetScanCodeForAction(CMD_LIST inpCmd, SDL_Scancode & outScanCode)
{
	for (int i = 0; i != HIGHOMEGA_MAXIMUM_SDLK; i++)
		if (KeyStates[0][i] == CMD_USE)
		{
			outScanCode = (SDL_Scancode)i;
			return true;
		}
	return false;
}
