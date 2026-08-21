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

#define HIGHOMEGA_SAVE_VERSION 0

namespace HIGHOMEGA
{
	namespace SERIALIZATION
	{
		// Non-default copy constructor makes a class non-trivially copyable, but doesn't always mean it's actually not trivially copyable
		// Therefore, over here we're hoping that T is actually trivially copyable.
		template<typename T>
		inline void serialize(const T& inPrim, std::vector<unsigned char>& outStream)
		{
			outStream.resize(outStream.size() + sizeof(T));
			memcpy(&(*(outStream.end() - sizeof(T))), (char*)(&inPrim), sizeof(T));
		}
		template<typename T>
		inline void deserialize(std::vector<unsigned char>& inStream, T& outPrim, unsigned int& streamIdx)
		{
			memcpy(&outPrim, inStream.data() + streamIdx, sizeof(T));
			streamIdx += sizeof(T);
		}
		template<>
		inline void serialize<std::string>(const std::string& inString, std::vector<unsigned char>& outStream)
		{
			serialize<unsigned int>((unsigned int)inString.length(), outStream);
			const std::vector<char> charVec(inString.begin(), inString.end());
			outStream.insert(outStream.end(), charVec.begin(), charVec.end());
		}
		template<>
		inline void deserialize<std::string>(std::vector<unsigned char>& inStream, std::string& outString, unsigned int& streamIdx)
		{
			unsigned int strLen;
			deserialize<unsigned int>(inStream, strLen, streamIdx);
			unsigned char* strPtr = new unsigned char[strLen + 1];
			memcpy(strPtr, inStream.data() + streamIdx, strLen);
			strPtr[strLen] = '\0';
			streamIdx += strLen;
			outString = std::string((char *)strPtr);
			delete[] strPtr;
		}
		template<typename T>
		inline void serializeVector(const std::vector<T>& inVector, std::vector<unsigned char>& outStream)
		{
			serialize<unsigned int>((unsigned int)inVector.size(), outStream); // 32 bit limit, we're ok with that
			for (unsigned int i = 0; i != inVector.size(); i++)
				serialize<T>(inVector[i], outStream);
		}
		template<typename T>
		inline void deserializeVector(std::vector<unsigned char>& inStream, std::vector<T>& outVector, unsigned int& streamIdx)
		{
			unsigned int vecSize;
			deserialize<unsigned int>(inStream, vecSize, streamIdx);
			outVector.resize(vecSize);
			for (unsigned int i = 0; i != outVector.size(); i++)
				deserialize<T>(inStream, outVector[i], streamIdx);
		}
	}
}