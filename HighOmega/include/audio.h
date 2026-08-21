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

#include "AL/al.h"
#include "AL/alc.h"
#include "util.h"
#include <mutex>
#include <list>
#include <string>

namespace HIGHOMEGA
{
	namespace AUDIO
	{
		class AudioSystemClass
		{
		public:
			enum SOUND_TYPE
			{
				MUSIC,
				SFX
			};
			static float globalGain;
			static float sfxGain;
			static float musicGain;

		private:
			ALCdevice* Device = nullptr;
			ALCcontext* Context = nullptr;
			class SoundBite : public CStyleWrapper
			{
			public:
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
				SOUND_TYPE cachedType;

				bool haveBuffer = false;
				bool haveSource = false;

				bool looping;
				bool done = false;

				void RemovePast();
				SoundBite();
				SoundBite(const std::string& fileName, SOUND_TYPE type, const vec3& pos, const vec3& vel, float amp, bool repeat = false);
				~SoundBite();
			};
			void RefreshLocalGains();

		public:
			struct audioSignalStruct {
				bool quit;
				std::mutex quit_mutex;
			};
			audioSignalStruct audioSignal;
			std::mutex AudioMutex;
			std::unordered_map<unsigned long long, SoundBite *> allSounds;

			static void AudioLoop(AudioSystemClass* audioSystem);
			unsigned long long Insert(const std::string& fileName, SOUND_TYPE type, const vec3& pos, const vec3& vel, float amp, bool repeat = false);
			void AdjustVolume(unsigned long long id, float newVolume);
			void SetSFXGain(float inSFXGain);
			void SetMusicGain(float inMusicGain);
			void SetGlobalGain(float inGlobalGain);
			void SetPosition(unsigned long long id, const vec3& newPos);
			void SetVelocity(unsigned long long id, const vec3& newVel);
			void Pause(unsigned long long id);
			void Resume(unsigned long long id);
			void Stop(unsigned long long id);
			void SetPlaybackSpeed(unsigned long long id, float inPitch);
			void SetListener(const vec3& pos, const vec3& vel, const vec3& fwd, const vec3& up);
		};
		extern AudioSystemClass AudioSystem;
	}
}