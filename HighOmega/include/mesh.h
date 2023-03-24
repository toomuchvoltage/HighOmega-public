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

#ifndef HIGHOMEGA_MESH_H
#define HIGHOMEGA_MESH_H

#include <string>
#include <vector>
#include <list>
#include <vmath.h>
#include <cctype>
#include "util.h"

namespace HIGHOMEGA
{
	namespace MESH
	{
		int readInt(const char *p);
		class DataElement
		{
		private:
			char mode;
			std::string sval;
			int ival;
			float fval;
			bool isNumber(const std::string& s, bool & isFloat);
		public:
			void fromString(std::string & inp);
			bool fromBinStream(char *stream, unsigned int *offset, std::vector<char> & cStringVector);
			void blobFromBinStream(char *stream, unsigned int *offset, unsigned int outpSize, std::vector<unsigned char> & outp);
			bool fvalue(float &out);
			bool ivalue(int &out);
			bool svalue(std::string &out);
			std::string & svalRef();
		};
		class DataBlock
		{
		public:
			std::vector<std::vector <DataElement>> rows;
			std::vector<unsigned char> blob;
			std::string tag, mode;
			DataBlock();
			DataBlock(std::string inpTag);
		};
		class DataGroup
		{
		public:
			std::string type;
			std::string name;
			std::vector<DataBlock> blocks;
			DataGroup(std::string inpType, std::string inpName);
		};
		class Mesh
		{
		private:
			// Path
			std::string path;

			// File content, size and offset in stream
			unsigned char *content;
			unsigned int content_size;
			unsigned int offset;

			// Helper functions for navigating file content
			bool eod();
			bool eod(unsigned int at_offset);
			bool moreData();
			std::string getString(std::vector<char> & cStringVector);
			std::string getString(bool &lineEnd, std::vector<char> & cStringVector);
		public:
			std::vector<DataGroup> DataGroups;

			std::string getPath();
			Mesh();
			Mesh(std::string path, std::string *prependMeshGroup = nullptr);
			~Mesh();

			static bool getDataRowString(DataGroup &inpGroup, std::string blockName, std::string rowName, std::string &out);
			static bool getDataRowString(DataBlock &inpBlock, std::string rowName, std::string &out);
			static bool getDataRowFloat(DataGroup &inpGroup, std::string blockName, std::string rowName, float &out);
			static bool getDataRowFloat(DataBlock &inpBlock, std::string rowName, float &out);
			static bool setDataRowFloat(DataBlock &inpBlock, std::string rowName, float &out);
			static bool getDataRowVec3(DataGroup &inpGroup, std::string blockName, std::string rowName, HIGHOMEGA::MATH::vec3 &out);
			static bool getDataRowVec3(DataBlock &inpBlock, std::string rowName, HIGHOMEGA::MATH::vec3 &out);
			static bool setDataRowVec3(DataBlock &inpBlock, std::string rowName, HIGHOMEGA::MATH::vec3 &in);
			static bool getDataRowMat4(DataGroup &inpGroup, std::string blockName, std::string rowName, HIGHOMEGA::MATH::mat4 &out);
			static bool getDataRowMat4(DataBlock &inpBlock, std::string rowName, HIGHOMEGA::MATH::mat4 &out);
			static bool setDataRowMat4(DataBlock &inpBlock, std::string rowName, HIGHOMEGA::MATH::mat4 &in);
			static bool getDataBlock(DataGroup &inpGroup, std::string inpTag, HIGHOMEGA::MESH::DataBlock **out);
		};
	}
}

#endif