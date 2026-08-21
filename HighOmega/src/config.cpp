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

#include "config.h"

bool HIGHOMEGA::CONFIG::ConfigINI::Load(const std::string& file)
{
	unsigned char* content;
	unsigned int contentSize;
	bool retVal = false;
	if (HIGHOMEGA::ResourceLoader::LoadFile(file, &content, contentSize) == HIGHOMEGA::ResourceLoader::FILE_LOAD_RESULT::FILE_LOAD_SUCCESS)
	{
		unsigned char* contentTmp = new unsigned char[contentSize + 1];
		memcpy(contentTmp, content, contentSize);
		delete content;
		contentTmp[contentSize] = '\0';
		std::string contentAsString = std::string((const char *)contentTmp);
		delete contentTmp;
		removeAll(contentAsString, "\r");
		std::vector<std::string> allLines;
		split(allLines, contentAsString, "\n");
		std::string curCategory = "";
		for (const std::string& curLine : allLines) {
			if (curLine.contains("=")) {
				std::vector<std::string> kvSplit;
				split(kvSplit, curLine, "=");
				std::string key = kvSplit[0];
				kvSplit.erase(kvSplit.begin());
				catKeyVal[curCategory][key] = join(kvSplit, "=");
			}
			else if (curLine.starts_with("[") && curLine.ends_with("]"))
			{
				curCategory = curLine;
			}
		}

		retVal = true;
	}
	return retVal;
}

void HIGHOMEGA::CONFIG::ConfigINI::Save(const std::string& file)
{
	std::string output;
	for (std::pair<const std::string, std::unordered_map<std::string, std::string>>& curCatKeyVal : catKeyVal)
	{
		output += curCatKeyVal.first + "\n";
		for (std::pair<const std::string, std::string>& curKeyVal : curCatKeyVal.second)
			output += curKeyVal.first + "=" + curKeyVal.second + "\n";
		output += "\n";
	}
	ResourceSaver::SaveBlob(file, (unsigned char *)output.data(), (unsigned int)output.length());
}

void HIGHOMEGA::CONFIG::ConfigINI::PopulateFromCommandMap()
{
	catKeyVal.erase("[input]");
	for (std::pair <const CMD_LIST, KEY_AND_STATE>& curCmd : CommandMap)
		catKeyVal["[input]"]["actionbind_" + StringVersion(curCmd.first)] = std::to_string(curCmd.second.key);
}

void HIGHOMEGA::CONFIG::ConfigINI::ExtractKeybBindings()
{
	if (catKeyVal.find("[input]") == catKeyVal.end()) return;
	for (std::pair<const std::string, std::string>& curKeyVal : catKeyVal["[input]"])
		if (curKeyVal.first.contains("actionbind_"))
		{
			std::string curKey = curKeyVal.first;
			replaceAll(curKey, "actionbind_", "");
			CMD_LIST curCmd = ActionFromString(curKey);
			if (curCmd != (CMD_LIST)-1)
				CommandMap[curCmd].key = atoi(curKeyVal.second.c_str());
		}
}

bool HIGHOMEGA::CONFIG::ConfigINI::CatAndKeyExist(const std::string& cat, const std::string& key)
{
	if (catKeyVal.find(cat) == catKeyVal.end()) return false;
	if (catKeyVal[cat].find(key) == catKeyVal[cat].end()) return false;
	return true;
}

bool HIGHOMEGA::CONFIG::ConfigINI::Eval(const std::string& cat, const std::string& key, std::string& out)
{
	if (!CatAndKeyExist(cat, key)) return false;
	out = catKeyVal[cat][key];
	return true;
}

bool HIGHOMEGA::CONFIG::ConfigINI::Eval(const std::string& cat, const std::string& key, int& out)
{
	if (!CatAndKeyExist(cat, key)) return false;
	out = atoi(catKeyVal[cat][key].c_str());
	return true;
}

bool HIGHOMEGA::CONFIG::ConfigINI::Eval(const std::string& cat, const std::string& key, unsigned int& out)
{
	if (!CatAndKeyExist(cat, key)) return false;
	out = (unsigned int)atoi(catKeyVal[cat][key].c_str());
	return true;
}

bool HIGHOMEGA::CONFIG::ConfigINI::Eval(const std::string& cat, const std::string& key, float& out)
{
	if (!CatAndKeyExist(cat, key)) return false;
	out = (float)atof(catKeyVal[cat][key].c_str());
	return true;
}