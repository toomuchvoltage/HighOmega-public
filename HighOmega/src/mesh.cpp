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

#include "mesh.h"

bool HIGHOMEGA::MESH::DataElement::isNumber(const std::string & s, bool & isFloat)
{
	unsigned int lenS = (unsigned int)s.length();
	if (!lenS) return false;
	unsigned char *sContent = (unsigned char *)s.c_str();

	for (int i = 0; i != lenS; i++)
	{
		unsigned char curChar = sContent[i];
		if ((curChar >= 48 && curChar <= 57) || curChar == '+' || curChar == '-') continue;
		if (curChar == '.')
		{
			isFloat = true;
			continue;
		}
		return false;
	}
	return true;
}

int HIGHOMEGA::MESH::readInt(const char * p)
{
	int x = 0;
	bool neg = false;
	if (*p == '-')
	{
		neg = true;
		++p;
	}
	while (*p != '\0')
	{
		x = (x * 10) + (*p - '0');
		++p;
	}
	if (neg)
	{
		x = -x;
	}
	return x;
}

void HIGHOMEGA::MESH::DataElement::fromString(std::string & inp)
{
	bool isFloat = false;
	if (isNumber(inp, isFloat))
	{
		if (!isFloat)
		{
			ival = readInt(inp.c_str());
			mode = 'i';
		}
		else
		{
			fval = strtof(inp.c_str(), nullptr);
			mode = 'f';
		}
	}
	else
	{
		sval = inp;
		mode = 's';
	}
}

bool HIGHOMEGA::MESH::DataElement::fromBinStream(char *stream, unsigned int *offset, std::vector<char>& cStringVector)
{
	char modeWithCase = stream[*offset];
	mode = (char)tolower((int)modeWithCase);
	bool endOfLine = false;
	if (mode != modeWithCase) endOfLine = true;
	(*offset)++;
	switch (mode)
	{
	case 'i':
		ival = *((int *)&stream[*offset]);
		*offset += sizeof(int);
		break;
	case 'f':
		fval = *((float *)&stream[*offset]);
		*offset += sizeof(float);
		break;
	default:
		unsigned int sLen = *((unsigned int *)&stream[*offset]);
		cStringVector.resize(sLen);
		*offset += sizeof(unsigned int);
		memcpy(cStringVector.data(), &stream[*offset], sLen);
		sval = std::string(cStringVector.begin(), cStringVector.end());
		*offset += sLen;
		break;
	}
	return endOfLine;
}

void HIGHOMEGA::MESH::DataElement::blobFromBinStream(char *stream, unsigned int *offset, unsigned int outpSize, std::vector<unsigned char> & outp)
{
	outp.resize(outpSize);
	memcpy(outp.data(), &stream[*offset], outpSize);
	*offset += outpSize;
}

bool HIGHOMEGA::MESH::DataElement::fvalue(float &out)
{
	if (mode != 'f') return false;
	out = fval;
	return true;
}

bool HIGHOMEGA::MESH::DataElement::ivalue(int &out)
{
	if (mode != 'i') return false;
	out = ival;
	return true;
}

bool HIGHOMEGA::MESH::DataElement::svalue(std::string &out)
{
	if (mode != 's') return false;
	out = sval;
	return true;
}

std::string & HIGHOMEGA::MESH::DataElement::svalRef()
{
	return sval;
}


HIGHOMEGA::MESH::DataBlock::DataBlock()
{
}

HIGHOMEGA::MESH::DataBlock::DataBlock(std::string inpTag)
{
	tag = inpTag;
}

bool HIGHOMEGA::MESH::Mesh::eod()
{
	return offset >= content_size;
}

bool HIGHOMEGA::MESH::Mesh::eod(unsigned int at_offset)
{
	return at_offset >= content_size;
}

bool HIGHOMEGA::MESH::Mesh::moreData()
{
	unsigned int cur_offset = offset;
	while (!eod(cur_offset))
	{
		if (content[cur_offset] == ' ' || content[cur_offset] == '\r' || content[cur_offset] == '\n')
			cur_offset++;
		else
			return true;
	}
	return false;
}

std::string HIGHOMEGA::MESH::Mesh::getString(std::vector<char> & cStringVector)
{
	bool lineEnd;
	return getString(lineEnd, cStringVector);
}

std::string HIGHOMEGA::MESH::Mesh::getString(bool &lineEnd, std::vector<char> & cStringVector)
{
	while (true)
	{
		if (eod())
			throw std::runtime_error("No string left to get");
		if (content[offset] == ' ' || content[offset] == '\r' || content[offset] == '\n')
		{
			offset++;
			continue;
		}
		break;
	}

	unsigned int cur_offset = offset;

	while (true)
	{
		if ( eod(cur_offset) || content[cur_offset] == ' ' || content[cur_offset] == '\r' || content[cur_offset] == '\n' )
		{
			cStringVector.resize((cur_offset - offset) + 1);
			memcpy(cStringVector.data(), &content[offset], cur_offset - offset);
			cStringVector[cur_offset - offset] = '\0';
			std::string ret_string = std::string(cStringVector.data());
			offset = cur_offset;
			if (content[cur_offset] == ' ')
				lineEnd = false;
			else
				lineEnd = true;
			return ret_string;
		}
		cur_offset++;
	}
}

bool HIGHOMEGA::MESH::Mesh::getDataRowString(DataGroup & inpGroup, std::string blockName, std::string rowName, std::string &out)
{
	DataBlock *propsBlock;
	if (!getDataBlock(inpGroup, blockName, &propsBlock)) return false;
	for (int i = 0; i != propsBlock->rows.size(); i++)
	{
		bool fetchRes = propsBlock->rows[i][0].svalue(out);
		if (!fetchRes) return false;
		if (out == rowName)
		{
			fetchRes = propsBlock->rows[i][1].svalue(out);
			if (!fetchRes) return false;
			return true;
		}
	}
	return false;
}

bool HIGHOMEGA::MESH::Mesh::getDataRowString(DataBlock & inpBlock, std::string rowName, std::string & out)
{
	for (int i = 0; i != inpBlock.rows.size(); i++)
	{
		bool fetchRes = inpBlock.rows[i][0].svalue(out);
		if (!fetchRes) return false;
		if (out == rowName)
		{
			fetchRes = inpBlock.rows[i][1].svalue(out);
			if (!fetchRes) return false;
			return true;
		}
	}
	return false;
}

bool HIGHOMEGA::MESH::Mesh::getDataRowFloat(DataGroup & inpGroup, std::string blockName, std::string rowName, float &out)
{
	DataBlock *propsBlock;
	if (!getDataBlock(inpGroup, blockName, &propsBlock)) return false;
	std::string outTmp;
	for (int i = 0; i != propsBlock->rows.size(); i++)
	{
		bool fetchRes = propsBlock->rows[i][0].svalue(outTmp);
		if (!fetchRes) return false;
		if (outTmp == rowName)
		{
			fetchRes = propsBlock->rows[i][1].fvalue(out);
			if (!fetchRes) return false;
			return true;
		}
	}
	return false;
}

bool HIGHOMEGA::MESH::Mesh::getDataRowFloat(DataBlock & inpBlock, std::string rowName, float & out)
{
	std::string outTmp;
	for (int i = 0; i != inpBlock.rows.size(); i++)
	{
		bool fetchRes = inpBlock.rows[i][0].svalue(outTmp);
		if (!fetchRes) return false;
		if (outTmp == rowName)
		{
			fetchRes = inpBlock.rows[i][1].fvalue(out);
			if (!fetchRes) return false;
			return true;
		}
	}
	return false;
}

bool HIGHOMEGA::MESH::Mesh::setDataRowFloat(DataBlock & inpBlock, std::string rowName, float & in)
{
	std::string outTmp, tmpStr;
	for (int i = 0; i != inpBlock.rows.size(); i++)
	{
		bool fetchRes = inpBlock.rows[i][0].svalue(outTmp);
		if (!fetchRes) return false;
		if (outTmp == rowName)
		{
			float curVal;
			fetchRes = inpBlock.rows[i][1].fvalue(curVal);
			if (!fetchRes) return false;
			tmpStr = std::to_string(in);
			inpBlock.rows[i][1].fromString(tmpStr);
			return true;
		}
	}
	return false;
}

bool HIGHOMEGA::MESH::Mesh::getDataRowVec3(DataGroup & inpGroup, std::string blockName, std::string rowName, HIGHOMEGA::MATH::vec3 &out)
{
	DataBlock *propsBlock;
	if (!getDataBlock(inpGroup, blockName, &propsBlock)) return false;
	std::string outTmp;
	for (int i = 0; i != propsBlock->rows.size(); i++)
	{
		bool fetchRes = propsBlock->rows[i][0].svalue(outTmp);
		if (!fetchRes) return false;
		if (outTmp == rowName)
		{
			float xFetch, yFetch, zFetch;
			fetchRes = propsBlock->rows[i][1].fvalue(xFetch);
			if (!fetchRes) return false;
			fetchRes = propsBlock->rows[i][2].fvalue(yFetch);
			if (!fetchRes) return false;
			fetchRes = propsBlock->rows[i][3].fvalue(zFetch);
			if (!fetchRes) return false;
			out.x = xFetch;
			out.y = yFetch;
			out.z = zFetch;
			return true;
		}
	}
	return false;
}

bool HIGHOMEGA::MESH::Mesh::getDataRowVec3(DataBlock & inpBlock, std::string rowName, HIGHOMEGA::MATH::vec3 & out)
{
	std::string outTmp;
	for (int i = 0; i != inpBlock.rows.size(); i++)
	{
		bool fetchRes = inpBlock.rows[i][0].svalue(outTmp);
		if (!fetchRes) return false;
		if (outTmp == rowName)
		{
			float xFetch, yFetch, zFetch;
			fetchRes = inpBlock.rows[i][1].fvalue(xFetch);
			if (!fetchRes) return false;
			fetchRes = inpBlock.rows[i][2].fvalue(yFetch);
			if (!fetchRes) return false;
			fetchRes = inpBlock.rows[i][3].fvalue(zFetch);
			if (!fetchRes) return false;
			out.x = xFetch;
			out.y = yFetch;
			out.z = zFetch;
			return true;
		}
	}
	return false;
}

bool HIGHOMEGA::MESH::Mesh::setDataRowVec3(DataBlock & inpBlock, std::string rowName, HIGHOMEGA::MATH::vec3 & in)
{
	std::string outTmp, tmpStr;
	for (int i = 0; i != inpBlock.rows.size(); i++)
	{
		bool fetchRes = inpBlock.rows[i][0].svalue(outTmp);
		if (!fetchRes) return false;
		if (outTmp == rowName)
		{
			float xFetch, yFetch, zFetch;
			fetchRes = inpBlock.rows[i][1].fvalue(xFetch);
			if (!fetchRes) return false;
			fetchRes = inpBlock.rows[i][2].fvalue(yFetch);
			if (!fetchRes) return false;
			fetchRes = inpBlock.rows[i][3].fvalue(zFetch);
			if (!fetchRes) return false;
			tmpStr = std::to_string(in.x);
			inpBlock.rows[i][1].fromString(tmpStr);
			tmpStr = std::to_string(in.y);
			inpBlock.rows[i][2].fromString(tmpStr);
			tmpStr = std::to_string(in.z);
			inpBlock.rows[i][3].fromString(tmpStr);
			return true;
		}
	}
	return false;
}

bool HIGHOMEGA::MESH::Mesh::getDataRowMat4(DataGroup & inpGroup, std::string blockName, std::string rowName, HIGHOMEGA::MATH::mat4 &out)
{
	DataBlock *propsBlock;
	if (!getDataBlock(inpGroup, blockName, &propsBlock)) return false;
	std::string outTmp;
	for (int i = 0; i != propsBlock->rows.size(); i++)
	{
		bool fetchRes = propsBlock->rows[i][0].svalue(outTmp);
		if (!fetchRes) return false;
		if (outTmp == rowName)
		{
			mat4 retVal;
			for (unsigned int j = 1; j != 17; j++) {
				unsigned int j_1 = j - 1;
				if (!propsBlock->rows[i][j].fvalue(retVal.i[j_1 / 4][j_1 % 4])) return false;
			}
			out = retVal;
			return true;
		}
	}
	return false;
}

bool HIGHOMEGA::MESH::Mesh::getDataRowMat4(DataBlock & inpBlock, std::string rowName, HIGHOMEGA::MATH::mat4 & out)
{
	std::string outTmp;
	for (int i = 0; i != inpBlock.rows.size(); i++)
	{
		bool fetchRes = inpBlock.rows[i][0].svalue(outTmp);
		if (!fetchRes) return false;
		if (outTmp == rowName)
		{
			mat4 retVal;
			for (unsigned int j = 1; j != 17; j++) {
				unsigned int j_1 = j - 1;
				if (!inpBlock.rows[i][j].fvalue(retVal.i[j_1 / 4][j_1 % 4])) return false;
			}
			out = retVal;
			return true;
		}
	}
	return false;
}

bool HIGHOMEGA::MESH::Mesh::setDataRowMat4(DataBlock & inpBlock, std::string rowName, HIGHOMEGA::MATH::mat4 & in)
{
	std::string outTmp, tmpStr;
	for (int i = 0; i != inpBlock.rows.size(); i++)
	{
		bool fetchRes = inpBlock.rows[i][0].svalue(outTmp);
		if (!fetchRes) return false;
		if (outTmp == rowName)
		{
			mat4 retVal;
			for (unsigned int j = 1; j != 17; j++) {
				unsigned int j_1 = j - 1;
				if (!inpBlock.rows[i][j].fvalue(retVal.i[j_1 / 4][j_1 % 4])) return false;
			}
			for (unsigned int j = 1; j != 17; j++) {
				unsigned int j_1 = j - 1;
				tmpStr = std::to_string(in.i[j_1 / 4][j_1 % 4]);
				inpBlock.rows[i][j].fromString(tmpStr);
			}
			return true;
		}
	}
	return false;
}

bool HIGHOMEGA::MESH::Mesh::getDataBlock(DataGroup & inpGroup, std::string inpTag, HIGHOMEGA::MESH::DataBlock **out)
{
	for (int i = 0; i != inpGroup.blocks.size(); i++)
		if (inpGroup.blocks[i].tag == inpTag)
		{
			*out = &inpGroup.blocks[i];
			return true;
		}
	return false;
}

std::string HIGHOMEGA::MESH::Mesh::getPath()
{
	return path;
}

HIGHOMEGA::MESH::Mesh::Mesh()
{
	content = nullptr;
}

HIGHOMEGA::MESH::Mesh::Mesh(std::string inpPath, std::string *prependMeshGroup)
{
	path = inpPath;

	offset = 0;
	HIGHOMEGA::ResourceLoader::LOAD_LOCATION loadLocation;
	HIGHOMEGA::ResourceLoader::LOAD_CHOSEN_ASSET loadChosenAsset;
	if ( HIGHOMEGA::ResourceLoader::Load("", path, &content, content_size, loadLocation, loadChosenAsset) != HIGHOMEGA::ResourceLoader::RESOURCE_LOAD_RESULT::RESOURCE_LOAD_SUCCESS )
		throw std::runtime_error("Could not load mesh file");

	std::vector<char> cStringVector;

	std::vector<DataElement> row;
	DataElement newDataElem;
	DataBlock newBlock("");
	DataGroup pGroup("", "");

	newDataElem.fromBinStream((char *)content, &offset, cStringVector);
	if (newDataElem.svalRef() != std::string("HIGHOMEGA_3MD")) throw std::runtime_error("Not a 3MD file");
	newDataElem.fromBinStream((char *)content, &offset, cStringVector);
	if (newDataElem.svalRef() != std::string("3.0")) throw std::runtime_error("Not 3MD version 3.0");

	while (moreData())
	{
		newDataElem.fromBinStream((char *)content, &offset, cStringVector);
		pGroup.type = newDataElem.svalRef();
		newDataElem.fromBinStream((char *)content, &offset, cStringVector);
		pGroup.name = newDataElem.svalRef();
		if (prependMeshGroup) pGroup.name = *prependMeshGroup + pGroup.name;
		pGroup.blocks.clear();

		while (true)
		{
			std::string blockTitle, blockMode;
			newDataElem.fromBinStream((char *)content, &offset, cStringVector);
			blockTitle = newDataElem.svalRef();
			if (blockTitle == std::string("-1")) break;
			newBlock.tag = blockTitle;
			newBlock.rows.clear();
			newBlock.blob.clear();
			int rowCount;
			newDataElem.fromBinStream((char *)content, &offset, cStringVector);
			newDataElem.svalue(blockMode);
			newBlock.mode = blockMode;
			newDataElem.fromBinStream((char *)content, &offset, cStringVector);
			newDataElem.ivalue(rowCount);
			if (blockMode == std::string("rows"))
			{
				for (int i = 0; i != rowCount;)
				{
					row.clear();

					while (true)
					{
						bool lineEnd = newDataElem.fromBinStream((char *)content, &offset, cStringVector);
						row.emplace_back(newDataElem);
						if (lineEnd)
						{
							i++;
							break;
						}
					}

					newBlock.rows.push_back(row);
				}
			}
			else
			{
				newDataElem.blobFromBinStream((char *)content, &offset, *((unsigned int *)&rowCount), newBlock.blob);
			}

			pGroup.blocks.push_back(newBlock);
		}

		DataGroups.push_back(pGroup);
	}

	delete[] content;
	content = nullptr;
}

HIGHOMEGA::MESH::Mesh::~Mesh()
{
	if (content) delete[] content;
	content = nullptr;
}

HIGHOMEGA::MESH::DataGroup::DataGroup(std::string inpType, std::string inpName)
{
	type = inpType;
	name = inpName;
}