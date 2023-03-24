#include <windows.h>
#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <map>
#include <string>
#include <algorithm>

HANDLE find_han;
bool didonce = false;

bool ReturnAFile (char *name)
{
	WIN32_FIND_DATAA ffd;
	if ( didonce == false )
	{
		find_han = FindFirstFileA ("*.spv",&ffd);
		strcpy(name,(char *)ffd.cFileName);
		didonce = true;
		return true;
	}
	if ( FindNextFileA(find_han, &ffd) == 0 )
		return false;
	else
	{
		strcpy(name,(const char *)ffd.cFileName);
		return true;
	}
}

FILE *outfile;
std::map <std::string, std::string> shaderMap;
std::map <std::string, unsigned int> shaderSizeMap;

void DisplayAsCode (char *name)
{
	char var_name[100];
	int name_len = strlen (name);
	strcpy (var_name,name);
	std::string keyName = std::string (name);
	std::transform(keyName.begin(), keyName.end(), keyName.begin(), ::tolower);
	for (int i = 0; i != strlen(var_name); i++)
	{
		if (var_name[i] == '.') var_name[i] = '_';
	}
	shaderMap[keyName] = std::string(var_name);
	fprintf (outfile, "static unsigned char %s[] = {\n",var_name);

	int counter = 0;
	unsigned int fileSize = 0;
	FILE *infile = fopen (name,"rb");
	bool firstWrite = true;
	while (!feof(infile))
	{
		char save = (unsigned char)((unsigned int)fgetc (infile));
		if ( !feof(infile) )
		{
			fileSize++;
			if (firstWrite)
			{
				fprintf(outfile, "0x%02X", (unsigned char)((unsigned int)~save));
				firstWrite = false;
			}
			else
				fprintf(outfile, ",0x%02X", (unsigned char)((unsigned int)~save));
		}
		else
			fprintf (outfile, "};\n");
		counter++;
		if ( counter == 20 )
		{
			fprintf(outfile, "\n");
			counter = 0;
		}
	}
	fclose(infile);

	shaderSizeMap[keyName] = fileSize;
}

int main(int argc,char *argv[])
{
	SetFileApisToANSI ();
	char fname[400];
	outfile = fopen("encodedshaders.h", "wb");
	fprintf(outfile, "#ifndef ENCODED_SHADERS_H\n");
	fprintf(outfile, "#define ENCODED_SHADERS_H\n\n");
	fprintf(outfile, "#include <unordered_map>\n");
	fprintf(outfile, "#include <string>\n\n");
	fprintf(outfile, "namespace HIGHOMEGA {\n");
	fprintf(outfile, "namespace ENCODED_SHADERS {\n");
	for (;;)
	{
		bool info = ReturnAFile (fname);
		if ( info == true )
		{
			DisplayAsCode (fname);
			fprintf (outfile, "\n");
		}
		else
			break;
	}
	fprintf(outfile, "static std::unordered_map<std::string, unsigned char *> shaderMap = {\n");
	unsigned int lastItem = shaderMap.size();
	unsigned int countItems = 0;
	for (std::pair <const std::string, std::string> & keyPair : shaderMap)
	{
		fprintf(outfile, "{\"shaders/%s\", %s}", keyPair.first.c_str(), keyPair.second.c_str());
		countItems++;
		if (countItems != lastItem) fprintf(outfile, ",");
		fprintf(outfile, "\n");
	}
	fprintf(outfile, "};\n");

	fprintf(outfile, "static std::unordered_map<std::string, unsigned int> shaderSizeMap = {\n");
	lastItem = shaderSizeMap.size();
	countItems = 0;
	for (std::pair <const std::string, unsigned int> & keyPair : shaderSizeMap)
	{
		fprintf(outfile, "{\"shaders/%s\", %d}", keyPair.first.c_str(), keyPair.second);
		countItems++;
		if (countItems != lastItem) fprintf(outfile, ",");
		fprintf(outfile, "\n");
	}
	fprintf(outfile, "};\n");

	fprintf(outfile, "}\n");
	fprintf(outfile, "}\n");
	fprintf(outfile, "#endif\n");
	fclose(outfile);

	system("move encodedshaders.h ..\\include");
	return 0;
}