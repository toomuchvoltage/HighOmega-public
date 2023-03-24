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

#ifndef HIGHOMEGA_AUDIO_H
#define HIGHOMEGA_AUDIO_H

#include "AL/al.h"
#include "AL/alc.h"
#include "util.h"
#include <shared_mutex>
#include <list>
#include <string>

namespace HIGHOMEGA
{
	namespace AUDIO
	{
		struct audioSignalStruct {
			bool quit;
			std::mutex quit_mutex;
		};
		extern audioSignalStruct audioSignal;
		extern std::shared_mutex AudioSharedMutex;
		void AudioLoop();
		class SoundBite : public CStyleWrapper
		{
		private:
			static ALCdevice* Device;
			static ALCcontext* Context;

			ALenum format;
			ALsizei size;
			ALvoid* data = nullptr;
			ALsizei freq;
			ALboolean loop;

			ALuint Buffer;
			ALuint Source;
			ALfloat SourcePos[3];
			ALfloat SourceVel[3];
			ALfloat gain;

			bool haveBuffer = false;
			bool haveSource = false;

			bool looping;
			unsigned long long id;
			bool done = false;
			void RemovePast();

		public:

			SoundBite();
			SoundBite(std::string fileName, float x, float y, float z, float vx, float vy, float vz, float amp, bool repeat = false);
			~SoundBite();
			void AdjustVolume(float newVolume);
			void SetPosition(float newX, float newY, float newZ);
			void SetVelocity(float newVX, float newVY, float newVZ);
			void Process();
			bool IsDone();
			void Pause();
			void Resume();
			void Stop();
			unsigned long long GetID();
			static void DestroyAudioSystem();
		};
		extern std::list <std::unique_ptr<HIGHOMEGA::AUDIO::SoundBite>> allSounds;
	}
}

#endif