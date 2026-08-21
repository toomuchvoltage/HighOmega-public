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

#include "audio.h"

HIGHOMEGA::AUDIO::AudioSystemClass HIGHOMEGA::AUDIO::AudioSystem;
float HIGHOMEGA::AUDIO::AudioSystemClass::globalGain = 1.0f;
float HIGHOMEGA::AUDIO::AudioSystemClass::sfxGain = 1.0f;
float HIGHOMEGA::AUDIO::AudioSystemClass::musicGain = 1.0f;

void HIGHOMEGA::AUDIO::AudioSystemClass::SoundBite::RemovePast()
{
	if (data) delete[] data;
	data = nullptr;

	if (!AudioSystem.Device || !AudioSystem.Context) return ;
	if (haveSource) alDeleteSources(1, &Source);
	if (haveBuffer) alDeleteBuffers(1, &Buffer);

	haveBuffer = false;
	haveSource = false;
}

HIGHOMEGA::AUDIO::AudioSystemClass::SoundBite::SoundBite()
{
	if (!AudioSystem.Device || !AudioSystem.Context) return;
	haveBuffer = false;
	haveSource = false;
	data = nullptr;
}

HIGHOMEGA::AUDIO::AudioSystemClass::SoundBite::SoundBite(const std::string& fileName, SOUND_TYPE type, const vec3& pos, const vec3& vel, float amp, bool repeat)
{
	short int numChannels;
	short int numBits;

	int waitLogCounter = 0;
	while ((!AudioSystem.Device || !AudioSystem.Context))
	{
		if (waitLogCounter % 10 == 0) LOG() << "Waiting for OAL device/context to be ready...";
		waitLogCounter = (waitLogCounter + 1) % 10;
	}

	for (int i = 0 ; i != 100; i++)
	{
		alGenBuffers(1, &Buffer);
		if (alGetError() != AL_NO_ERROR) { RemovePast(); LOG() << "Could not create sound buffer"; continue; }
		haveBuffer = true;

		unsigned char* fileContent = nullptr;
		unsigned int contentSize;
		ResourceLoader::LOAD_LOCATION loadLocation;
		if (ResourceLoader::Load("", fileName, &fileContent, contentSize, loadLocation) == ResourceLoader::RESOURCE_LOAD_RESULT::RESOURCE_LOAD_SUCCESS)
		{
			ResourceLoader::WAVLoad(fileContent, numChannels, numBits, (void**)&data, (unsigned int&)size, (unsigned int&)freq);
			delete[] fileContent;
		}
		else
			FATAL_ERROR("Sound file not found");
		if (numChannels == 1 && numBits == 8) format = AL_FORMAT_MONO8;
		if (numChannels == 1 && numBits == 16) format = AL_FORMAT_MONO16;
		if (numChannels == 2 && numBits == 8) format = AL_FORMAT_STEREO8;
		if (numChannels == 2 && numBits == 16) format = AL_FORMAT_STEREO16;

		alBufferData(Buffer, format, data, size, freq);
		if (alGetError() != AL_NO_ERROR) { RemovePast(); LOG() << "Could not supply sound data"; continue; }
		delete[] data;
		data = nullptr;

		alGenSources(1, &Source);
		haveSource = true;

		SourcePos[0] = pos.x;
		SourcePos[1] = pos.y;
		SourcePos[2] = pos.z;
		SourceVel[0] = vel.x;
		SourceVel[1] = vel.y;
		SourceVel[2] = vel.z;
		cachedType = type;
		gain = amp;
		float finalGain = gain * ((cachedType == SOUND_TYPE::SFX) ? AudioSystemClass::sfxGain : AudioSystemClass::musicGain);

		if (alGetError() != AL_NO_ERROR) { RemovePast(); LOG() << "Could not create sound source"; continue; }

		alSourcei(Source, AL_BUFFER, Buffer);
		alSourcef(Source, AL_PITCH, 1.0);
		alSourcef(Source, AL_MAX_GAIN, 10.0f);
		alSourcef(Source, AL_GAIN, finalGain);
		alSourcefv(Source, AL_POSITION, SourcePos);
		alSourcefv(Source, AL_VELOCITY, SourceVel);
		alSourcei(Source, AL_LOOPING, repeat ? AL_TRUE : AL_FALSE);

		alSourcePlay(Source);

		return;
	}
}

HIGHOMEGA::AUDIO::AudioSystemClass::SoundBite::~SoundBite()
{
	RemovePast();
}

void HIGHOMEGA::AUDIO::AudioSystemClass::RefreshLocalGains()
{
	for (std::pair<const unsigned long long, SoundBite*>& curSound : allSounds)
	{
		float finalGain = curSound.second->gain * ((curSound.second->cachedType == SOUND_TYPE::SFX) ? AudioSystemClass::sfxGain : AudioSystemClass::musicGain);
		alSourcef(curSound.second->Source, AL_GAIN, finalGain);
	}
}

void HIGHOMEGA::AUDIO::AudioSystemClass::AudioLoop(AudioSystemClass* audioSystem)
{
	{ std::lock_guard<std::mutex> lk(audioSystem->AudioMutex);
	audioSystem->Device = alcOpenDevice(NULL);
	audioSystem->Context = alcCreateContext(audioSystem->Device, NULL);
	alcMakeContextCurrent(audioSystem->Context); }

	for (;;)
	{
		TimerObject timerObj;
		timerObj.Start();

		std::unique_lock<std::mutex> lk(audioSystem->AudioMutex, std::defer_lock);
		lk.lock();
		try
		{
			for (std::pair<const unsigned long long, SoundBite *>& curSound : audioSystem->allSounds)
			{
				ALint play;
				alGetSourcei(curSound.second->Source, AL_SOURCE_STATE, &play);
				if (play != AL_PLAYING && play != AL_PAUSED) curSound.second->done = true;
			}
			std::unordered_map<unsigned long long, SoundBite *>::iterator curSound = audioSystem->allSounds.begin();
			while (curSound != audioSystem->allSounds.end())
			{
				SoundBite* soundPtr = (*curSound).second;
				if (soundPtr->done)
				{
					delete soundPtr;
					curSound = audioSystem->allSounds.erase(curSound);
				}
				else
					curSound++;
			}
		}
		catch (...)
		{
			lk.unlock();
			break;
		}
		lk.unlock();

		double timerDiff = 10.0 - timerObj.Diff() * 1000.0;
		if (timerDiff > 0.0) std::this_thread::sleep_for(std::chrono::milliseconds((int)timerDiff));
		else std::this_thread::sleep_for(std::chrono::milliseconds(1));

		bool readQuit;
		{ std::unique_lock<std::mutex> lk(audioSystem->audioSignal.quit_mutex); readQuit = audioSystem->audioSignal.quit; }
		if (readQuit) break;
	}

	{ std::lock_guard<std::mutex> lk(AudioSystem.AudioMutex);
	for (std::pair<const unsigned long long, SoundBite*>& curSound : audioSystem->allSounds)
	{
		alSourceStop(curSound.second->Source);
		delete curSound.second;
	}
	audioSystem->allSounds.clear();

	alcMakeContextCurrent(NULL);
	alcDestroyContext(audioSystem->Context);
	alcCloseDevice(audioSystem->Device);

	audioSystem->Context = nullptr;
	audioSystem->Device = nullptr; }
}

unsigned long long HIGHOMEGA::AUDIO::AudioSystemClass::Insert(const std::string& fileName, SOUND_TYPE type, const vec3& pos, const vec3& vel, float amp, bool repeat)
{
	unsigned long long id = threadSafeMersenneTwister64Bit();
	std::unique_lock<std::mutex> lk(AudioMutex, std::defer_lock);
	lk.lock();
	try
	{
		allSounds[id] = new SoundBite(fileName, type, pos, vel, amp, repeat);
	}
	catch (const std::runtime_error& e)
	{
		lk.unlock();
		if (!strcmp(e.what(), "Sound file not found")) throw;
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		lk.lock();
	}
	lk.unlock();
	return id;
}

void HIGHOMEGA::AUDIO::AudioSystemClass::AdjustVolume(unsigned long long id, float newVolume)
{
	std::lock_guard<std::mutex> lk(AudioMutex);
	if (!Device || !Context) return;
	if (allSounds.find(id) == allSounds.end()) return;
	float finalGain = allSounds[id]->gain = newVolume;
	if (allSounds[id]->cachedType == SOUND_TYPE::SFX) finalGain *= AudioSystemClass::sfxGain;
	alSourcef(allSounds[id]->Source, AL_GAIN, finalGain);
}

void HIGHOMEGA::AUDIO::AudioSystemClass::SetSFXGain(float inSFXGain)
{
	std::lock_guard<std::mutex> lk(AudioMutex);
	if (!Device || !Context) return;
	AudioSystemClass::sfxGain = inSFXGain;
	RefreshLocalGains();
}

void HIGHOMEGA::AUDIO::AudioSystemClass::SetMusicGain(float inMusicGain)
{
	std::lock_guard<std::mutex> lk(AudioMutex);
	if (!Device || !Context) return;
	AudioSystemClass::musicGain = inMusicGain;
	RefreshLocalGains();
}

void HIGHOMEGA::AUDIO::AudioSystemClass::SetGlobalGain(float inGlobalGain)
{
	std::lock_guard<std::mutex> lk(AudioMutex);
	if (!Device || !Context) return;
	AudioSystemClass::globalGain = inGlobalGain;
	alListenerf(AL_GAIN, AudioSystemClass::globalGain);
}

void HIGHOMEGA::AUDIO::AudioSystemClass::SetPosition(unsigned long long id, const vec3& newPos)
{
	std::lock_guard<std::mutex> lk(AudioMutex);
	if (!Device || !Context) return;
	if (allSounds.find(id) == allSounds.end()) return;
	allSounds[id]->SourcePos[0] = newPos.x;
	allSounds[id]->SourcePos[1] = newPos.y;
	allSounds[id]->SourcePos[2] = newPos.z;
	alSourcefv(allSounds[id]->Source, AL_POSITION, allSounds[id]->SourcePos);
}

void HIGHOMEGA::AUDIO::AudioSystemClass::SetVelocity(unsigned long long id, const vec3& newVel)
{
	std::lock_guard<std::mutex> lk(AudioMutex);
	if (!Device || !Context) return;
	if (allSounds.find(id) == allSounds.end()) return;
	allSounds[id]->SourceVel[0] = newVel.x;
	allSounds[id]->SourceVel[1] = newVel.y;
	allSounds[id]->SourceVel[2] = newVel.z;
	alSourcefv(allSounds[id]->Source, AL_VELOCITY, allSounds[id]->SourceVel);
}

void HIGHOMEGA::AUDIO::AudioSystemClass::Pause(unsigned long long id)
{
	std::lock_guard<std::mutex> lk(AudioMutex);
	if (!Device || !Context) return;
	if (allSounds.find(id) == allSounds.end()) return;
	alSourcePause(allSounds[id]->Source);
}

void HIGHOMEGA::AUDIO::AudioSystemClass::Resume(unsigned long long id)
{
	std::lock_guard<std::mutex> lk(AudioMutex);
	if (!Device || !Context) return;
	if (allSounds.find(id) == allSounds.end()) return;
	alSourcePlay(allSounds[id]->Source);
}

void HIGHOMEGA::AUDIO::AudioSystemClass::Stop(unsigned long long id)
{
	std::lock_guard<std::mutex> lk(AudioMutex);
	if (!Device || !Context) return;
	if (allSounds.find(id) == allSounds.end()) return;
	alSourceStop(allSounds[id]->Source);
}

void HIGHOMEGA::AUDIO::AudioSystemClass::SetPlaybackSpeed(unsigned long long id, float inPitch)
{
	std::lock_guard<std::mutex> lk(AudioMutex);
	if (!Device || !Context) return;
	if (allSounds.find(id) == allSounds.end()) return;
	alSourcef(allSounds[id]->Source, AL_PITCH, inPitch);
}

void HIGHOMEGA::AUDIO::AudioSystemClass::SetListener(const vec3& pos, const vec3& vel, const vec3& fwd, const vec3& up)
{
	std::lock_guard<std::mutex> lk(AudioMutex);
	if (!Device || !Context) return;

	ALfloat listenerPos[] = { pos.x, pos.y, pos.z };
	alListenerfv(AL_POSITION, listenerPos);

	ALfloat listenerVel[] = { vel.x, vel.y, vel.z };
	alListenerfv(AL_VELOCITY, listenerVel);

	ALfloat listenerOri[] = { fwd.x, fwd.y, fwd.z, up.x, up.y, up.z };
	alListenerfv(AL_ORIENTATION, listenerOri);
}