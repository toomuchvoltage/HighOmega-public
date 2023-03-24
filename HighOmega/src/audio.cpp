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

#include "audio.h"

HIGHOMEGA::AUDIO::audioSignalStruct HIGHOMEGA::AUDIO::audioSignal;
std::shared_mutex HIGHOMEGA::AUDIO::AudioSharedMutex;
std::list <std::unique_ptr<HIGHOMEGA::AUDIO::SoundBite>> HIGHOMEGA::AUDIO::allSounds;
ALCdevice *HIGHOMEGA::AUDIO::SoundBite::Device = nullptr;
ALCcontext *HIGHOMEGA::AUDIO::SoundBite::Context = nullptr;

void HIGHOMEGA::AUDIO::AudioLoop()
{
	for (;;)
	{
		TimerObject timerObj;
		timerObj.Start();

		AudioSharedMutex.lock_shared();
		try
		{
			if (allSounds.size() > 0) {
				std::list<std::unique_ptr<HIGHOMEGA::AUDIO::SoundBite>>::iterator curSound = allSounds.begin();
				while (curSound != allSounds.end())
				{
					SoundBite *soundPtr = (*curSound).get();
					soundPtr->Process();
					curSound++;
				}
				curSound = allSounds.begin();
				while (curSound != allSounds.end())
				{
					SoundBite *soundPtr = (*curSound).get();
					if (soundPtr->IsDone())
					{
						curSound = allSounds.erase(curSound);
					}
					else
						curSound++;
				}
			}
			else
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		catch (...)
		{
			AudioSharedMutex.unlock_shared();
			break;
		}
		AudioSharedMutex.unlock_shared();

		double timerDiff = 10.0 - timerObj.Diff()*1000.0;
		if (timerDiff > 0.0) std::this_thread::sleep_for(std::chrono::milliseconds((int)timerDiff));
		else std::this_thread::sleep_for(std::chrono::milliseconds(1));

		bool readQuit;
		{std::unique_lock<std::mutex> lk(audioSignal.quit_mutex); readQuit = audioSignal.quit; }
		if (readQuit) break;
	}

	for (std::unique_ptr<SoundBite> & curSound : allSounds)
		curSound.get()->Stop();
	allSounds.clear();

	SoundBite::DestroyAudioSystem();
}

void HIGHOMEGA::AUDIO::SoundBite::RemovePast()
{
	if (haveBuffer) alDeleteBuffers(1, &Buffer);
	if (haveSource) alDeleteSources(1, &Source);
	if (data) delete[] data;

	haveBuffer = false;
	haveSource = false;
	data = nullptr;
}

HIGHOMEGA::AUDIO::SoundBite::SoundBite()
{
	haveBuffer = false;
	haveSource = false;
	data = nullptr;
}

HIGHOMEGA::AUDIO::SoundBite::SoundBite(std::string fileName, float x, float y, float z, float vx, float vy, float vz, float amp, bool repeat)
{
	if (Context == nullptr)
	{
		Device = alcOpenDevice(NULL);
		Context = alcCreateContext(Device, NULL);
		alcMakeContextCurrent(Context);
	}

	short int numChannels;
	short int numBits;

	alGenBuffers(1, &Buffer);
	if (alGetError() != AL_NO_ERROR) { RemovePast(); throw std::runtime_error("Could not create sound buffer"); }
	haveBuffer = true;

	unsigned char *fileContent = nullptr;
	unsigned int contentSize;
	ResourceLoader::LOAD_LOCATION loadLocation;
	ResourceLoader::LOAD_CHOSEN_ASSET loadChosenAsset;
	if (ResourceLoader::Load("", fileName, &fileContent, contentSize, loadLocation, loadChosenAsset) == ResourceLoader::RESOURCE_LOAD_RESULT::RESOURCE_LOAD_SUCCESS)
	{
		ResourceLoader::WAVLoad(fileContent, numChannels, numBits, (void **)&data, (unsigned int &)size, (unsigned int &)freq);
		delete[] fileContent;
	}
	else
		throw std::runtime_error("Sound file not found");
	if (numChannels == 1 && numBits == 8) format = AL_FORMAT_MONO8;
	if (numChannels == 1 && numBits == 16) format = AL_FORMAT_MONO16;
	if (numChannels == 2 && numBits == 8) format = AL_FORMAT_STEREO8;
	if (numChannels == 2 && numBits == 16) format = AL_FORMAT_STEREO16;

	alBufferData(Buffer, format, data, size, freq);
	if (alGetError() != AL_NO_ERROR) { RemovePast(); throw std::runtime_error("Could not supply sound data"); }
	delete[] data;
	data = nullptr;

	alGenSources(1, &Source);
	haveSource = true;

	SourcePos[0] = x;
	SourcePos[1] = y;
	SourcePos[2] = z;
	SourceVel[0] = vx;
	SourceVel[1] = vy;
	SourceVel[2] = vz;
	gain = amp;

	if (alGetError() != AL_NO_ERROR) { RemovePast(); throw std::runtime_error("Could not create sound source"); }

	alSourcei(Source, AL_BUFFER, Buffer);
	alSourcef(Source, AL_PITCH, 1.0);
	alSourcef(Source, AL_GAIN, gain);
	alSourcefv(Source, AL_POSITION, SourcePos);
	alSourcefv(Source, AL_VELOCITY, SourceVel);
	alSourcei(Source, AL_LOOPING, repeat ? AL_TRUE : AL_FALSE);

	alSourcePlay(Source);

	id = mersenneTwister64BitPRNG();
}

HIGHOMEGA::AUDIO::SoundBite::~SoundBite()
{
	RemovePast();
}

void HIGHOMEGA::AUDIO::SoundBite::AdjustVolume(float newVolume)
{
	gain = newVolume;
	alSourcef(Source, AL_GAIN, gain);
}

void HIGHOMEGA::AUDIO::SoundBite::SetPosition(float newX, float newY, float newZ)
{
	SourcePos[0] = newX;
	SourcePos[1] = newY;
	SourcePos[2] = newZ;
	alSourcefv(Source, AL_POSITION, SourcePos);
}

void HIGHOMEGA::AUDIO::SoundBite::SetVelocity(float newVX, float newVY, float newVZ)
{
	SourceVel[0] = newVX;
	SourceVel[1] = newVY;
	SourceVel[2] = newVZ;
	alSourcefv(Source, AL_VELOCITY, SourceVel);
}

void HIGHOMEGA::AUDIO::SoundBite::Process()
{
	if (done) return;
	ALint play;
	alGetSourcei(Source, AL_SOURCE_STATE, &play);
	if (play != AL_PLAYING && play != AL_PAUSED)
	{
		done = true;
	}
}

bool HIGHOMEGA::AUDIO::SoundBite::IsDone()
{
	return done;
}

void HIGHOMEGA::AUDIO::SoundBite::Pause()
{
	alSourcePause(Source);
}

void HIGHOMEGA::AUDIO::SoundBite::Resume()
{
	alSourcePlay(Source);
}

void HIGHOMEGA::AUDIO::SoundBite::Stop()
{
	alSourceStop(Source);
}

unsigned long long HIGHOMEGA::AUDIO::SoundBite::GetID()
{
	return id;
}

void HIGHOMEGA::AUDIO::SoundBite::DestroyAudioSystem()
{
	if (Context != nullptr)
	{
		alcMakeContextCurrent(NULL);
		alcDestroyContext(SoundBite::Context);
		alcCloseDevice(SoundBite::Device);

		Context = nullptr;
		Device = nullptr;
	}
}
