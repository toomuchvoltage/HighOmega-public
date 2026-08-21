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

#include "events.h"

std::unordered_map <HIGHOMEGA::EVENTS::CMD_LIST, HIGHOMEGA::EVENTS::KEY_AND_STATE> HIGHOMEGA::EVENTS::CommandMap;
bool HIGHOMEGA::EVENTS::anyKeyDown = false, HIGHOMEGA::EVENTS::anyButtonDown = false;
bool HIGHOMEGA::EVENTS::leftMouseDown, HIGHOMEGA::EVENTS::midMouseDown, HIGHOMEGA::EVENTS::rightMouseDown;
std::unordered_map <std::string, int> HIGHOMEGA::EVENTS::pressedInputConsumers;
std::unordered_map <std::string, vec2> HIGHOMEGA::EVENTS::mouseMovementConsumers;
std::unordered_map <std::string, float> HIGHOMEGA::EVENTS::mouseWheelConsumers;
bool HIGHOMEGA::EVENTS::windowMinimized = false;

void HIGHOMEGA::EVENTS::TryPopulateDefaultKeybinds()
{
	// insert() will prevent overwriting anything loaded from config
	CommandMap.insert({ CMD_MOVE_FORWARD, { SDL_SCANCODE_W } });
	CommandMap.insert({ CMD_MOVE_BACKWARD, { SDL_SCANCODE_S } });
	CommandMap.insert({ CMD_MOVE_LEFT, { SDL_SCANCODE_A } });
	CommandMap.insert({ CMD_MOVE_RIGHT, { SDL_SCANCODE_D } });
	CommandMap.insert({ CMD_JUMP, { SDL_SCANCODE_SPACE } });
	CommandMap.insert({ CMD_CROUCH, { SDL_SCANCODE_LCTRL } });
	CommandMap.insert({ CMD_FIRE, { HIGHOMEGA_MOUSE_BUTTON_LEFT } });
	CommandMap.insert({ CMD_ALTFIRE, { HIGHOMEGA_MOUSE_BUTTON_RIGHT } });
	CommandMap.insert({ CMD_MEMREPORT, { SDL_SCANCODE_F2 } });
	CommandMap.insert({ CMD_QUICKSAVE, { SDL_SCANCODE_F5 } });
	CommandMap.insert({ CMD_QUICKLOAD, { SDL_SCANCODE_F9 } });
	CommandMap.insert({ CMD_USE, { SDL_SCANCODE_E } });
	CommandMap.insert({ CMD_HAMMER, { SDL_SCANCODE_1 } });
	CommandMap.insert({ CMD_AK47, { SDL_SCANCODE_2 } });
	CommandMap.insert({ CMD_RELOAD, { SDL_SCANCODE_R } });
	CommandMap.insert({ CMD_FLASHLIGHT, { SDL_SCANCODE_L } });
	CommandMap.insert({ CMD_MAIN_MENU, { SDL_SCANCODE_ESCAPE } });
	CommandMap.insert({ CMD_TABLET, { SDL_SCANCODE_M } });
	CommandMap.insert({ CMD_SWITCH_TO_PT_MODE, { SDL_SCANCODE_P } });
	CommandMap.insert({ CMD_QUICKWALK, { SDL_SCANCODE_Q } });
	CommandMap.insert({ CMD_WRIST_PEEK, { SDL_SCANCODE_Z } });
}

int SDLCALL HIGHOMEGA::EVENTS::Handler()
{
	static bool firstRun = true;

	if (firstRun)
	{
		SDL_Init(SDL_INIT_EVENTS);
		SDL_SetRelativeMouseMode(SDL_TRUE);

		TryPopulateDefaultKeybinds();
	}

	bool anyKeyDownLocal = false;
	SDL_Event events;
	while (SDL_PollEvent(&events))
	{
		switch (events.type)
		{
			case SDL_KEYDOWN:
			{
				anyKeyDownLocal = true;
				if (events.key.keysym.scancode != CommandMap[CMD_MAIN_MENU].key && events.key.keysym.scancode != CommandMap[CMD_MEMREPORT].key)
					for (std::pair<const std::string, int>& curConsumer : pressedInputConsumers)
						curConsumer.second = events.key.keysym.scancode;
				for (std::pair<const CMD_LIST, KEY_AND_STATE>& curCommandKeyState : CommandMap)
					if (curCommandKeyState.second.key == events.key.keysym.scancode)
					{
						curCommandKeyState.second.state = DOWN;
						break;
					}
				break;
			}
			case SDL_KEYUP:
			{
				for (std::pair<const CMD_LIST, KEY_AND_STATE>& curCommandKeyState : CommandMap)
					if (curCommandKeyState.second.key == events.key.keysym.scancode)
					{
						curCommandKeyState.second.state = UP;
						break;
					}
				break;
			}
			case SDL_MOUSEWHEEL:
			{
				for (std::pair<const std::string, float>& curConsumer : mouseWheelConsumers)
					curConsumer.second += (float)events.wheel.y;
				break;
			}
			case SDL_MOUSEBUTTONDOWN:
			{
				switch (events.button.button)
				{
					case SDL_BUTTON_LEFT:
					{
						for (std::pair<const std::string, int>& curConsumer : pressedInputConsumers)
							curConsumer.second = HIGHOMEGA_MOUSE_BUTTON_LEFT;
						leftMouseDown = true;
						break;
					}
					case SDL_BUTTON_MIDDLE:
					{
						for (std::pair<const std::string, int>& curConsumer : pressedInputConsumers)
							curConsumer.second = HIGHOMEGA_MOUSE_BUTTON_MIDDLE;
						midMouseDown = true;
						break;
					}
					case SDL_BUTTON_RIGHT:
					{
						for (std::pair<const std::string, int>& curConsumer : pressedInputConsumers)
							curConsumer.second = HIGHOMEGA_MOUSE_BUTTON_RIGHT;
						rightMouseDown = true;
						break;
					}
				}
				break;
			}
			case SDL_MOUSEBUTTONUP:
			{
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
			}
			case SDL_MOUSEMOTION:
			{
				for (std::pair<const std::string, vec2>& curConsumer : mouseMovementConsumers)
					curConsumer.second += vec2((float)events.motion.xrel, -(float)events.motion.yrel);
				break;
			}
			case SDL_WINDOWEVENT:
			{
				if (events.window.event == SDL_WINDOWEVENT_MINIMIZED)
					HIGHOMEGA::EVENTS::windowMinimized = true;
				else if (events.window.event == SDL_WINDOWEVENT_RESTORED)
					HIGHOMEGA::EVENTS::windowMinimized = false;
				break;
			}
		}
	}
	anyKeyDown = anyKeyDownLocal;
	anyButtonDown = anyKeyDown || leftMouseDown || midMouseDown || rightMouseDown;

	Uint8 mouse_state = SDL_GetMouseState(nullptr, nullptr);
	for (std::pair<const CMD_LIST, KEY_AND_STATE>& curCommandKeyState : CommandMap)
		if (curCommandKeyState.second.key == HIGHOMEGA_MOUSE_BUTTON_MIDDLE)
			curCommandKeyState.second.state = (mouse_state & SDL_BUTTON(SDL_BUTTON_MIDDLE)) ? DOWN : UP;
		else if (curCommandKeyState.second.key == HIGHOMEGA_MOUSE_BUTTON_LEFT)
			curCommandKeyState.second.state = (mouse_state & SDL_BUTTON(SDL_BUTTON_LEFT)) ? DOWN : UP;
		else if (curCommandKeyState.second.key == HIGHOMEGA_MOUSE_BUTTON_RIGHT)
			curCommandKeyState.second.state = (mouse_state & SDL_BUTTON(SDL_BUTTON_RIGHT)) ? DOWN : UP;

	return 1;
}

bool HIGHOMEGA::EVENTS::GetStateOfAction(CMD_LIST inpCmd)
{
	if (CommandMap.find(inpCmd) != CommandMap.end())
		return CommandMap[inpCmd].state == DOWN;
	return false;
}

bool HIGHOMEGA::EVENTS::ActionHasKey(CMD_LIST inpCmd)
{
	if (CommandMap.find(inpCmd) == CommandMap.end()) return false;
	return CommandMap[inpCmd].key != -1;
}

HIGHOMEGA::EVENTS::CMD_LIST HIGHOMEGA::EVENTS::ActionFromString(const std::string& inpCmd)
{
	for (std::pair<const std::string, HIGHOMEGA::EVENTS::CMD_LIST>& curNameToEnum : HIGHOMEGA::EVENTS::cmdNameToEnum)
		if (curNameToEnum.first == inpCmd)
			return curNameToEnum.second;
	return (HIGHOMEGA::EVENTS::CMD_LIST)-1;
}

std::string HIGHOMEGA::EVENTS::GetKeynameForAction(CMD_LIST inpCmd)
{
	if (CommandMap.find(inpCmd) == CommandMap.end() || CommandMap[inpCmd].key == -1) return "Unassigned";
	if (CommandMap[inpCmd].key == HIGHOMEGA_MOUSE_BUTTON_MIDDLE) return "Middle mouse";
	else if (CommandMap[inpCmd].key == HIGHOMEGA_MOUSE_BUTTON_LEFT) return "Left mouse";
	else if (CommandMap[inpCmd].key == HIGHOMEGA_MOUSE_BUTTON_RIGHT) return "Right mouse";
	else return std::string(SDL_GetScancodeName((SDL_Scancode)CommandMap[inpCmd].key));
}

std::string HIGHOMEGA::EVENTS::StringVersion(CMD_LIST inpCmd)
{
	for (std::pair<const std::string, CMD_LIST>& curNameToEnum : cmdNameToEnum)
		if (inpCmd == curNameToEnum.second) return curNameToEnum.first;
	return "";
}

std::string HIGHOMEGA::EVENTS::GetDisplayFriendlyName(CMD_LIST inpCmd)
{
	std::string friendlyCmdName = StringVersion(inpCmd);
	if (friendlyCmdName == "") return "";
	replaceAll(friendlyCmdName, "CMD_", "");
	friendlyCmdName = toLowerCase(friendlyCmdName);
	replaceAll(friendlyCmdName, "_", " ");
	friendlyCmdName[0] = std::toupper(static_cast<unsigned char>(friendlyCmdName[0]));
	return friendlyCmdName;
}