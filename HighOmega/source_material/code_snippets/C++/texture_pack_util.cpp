
HIGHOMEGA::Packer::Packer()
{
	width = 0;
	height = 0;
}

void HIGHOMEGA::Packer::resize(unsigned int newWidth, unsigned int newHeight)
{
	if (width == newWidth && height == newHeight) return;
	if (newWidth < width || newHeight < height) throw ("Both new width and height should be larger");

	std::vector <vec3> newCSpace;

	newCSpace.resize(newWidth * newHeight);
	for (int i = 0; i != newWidth * newHeight; i++)
		newCSpace[i] = vec3(0.0f);

	for (int i = 0; i != height; i++)
		for (int j = 0; j != width; j++)
		{
			int curOldIndex = (i * width) + j;
			int curNewIndex = (i * newWidth) + j;
			newCSpace[curNewIndex] = cSpace[curOldIndex];
		}

	cSpace = newCSpace;
	width = newWidth;
	height = newHeight;
}

void HIGHOMEGA::Packer::scanForEmpty(unsigned int inpWidth, unsigned int inpHeight, vec2 & emptyStart, vec2 & additionalDims)
{
	unsigned int minimalNewAreaRequired = 0xFFFFFFFF;

	for (int j = 0; j < (int)height + 1; j++)
		for (int i = 0; i < (int)width + 1; i++)
		{
			bool foundEmptySpace = true;
			for (int y = 0; y != inpHeight; y++)
				for (int x = 0; x != inpWidth; x++)
				{
					int checkX = i + x;
					int checkY = j + y;
					int curIndex = (checkY * width) + checkX;
					if (curIndex < (int)cSpace.size() && cSpace[curIndex] != vec3(0.0))
					{
						foundEmptySpace = false;
						break;
					}
				}

			unsigned int potentialNewWidth = width;
			if (i + inpWidth > width) potentialNewWidth = i + inpWidth;
			unsigned int potentialNewHeight = height;
			if (j + inpHeight > height) potentialNewHeight = j + inpHeight;
			unsigned int newRequiredArea = potentialNewWidth * potentialNewHeight;

			if (foundEmptySpace && newRequiredArea < minimalNewAreaRequired)
			{
				minimalNewAreaRequired = newRequiredArea;
				emptyStart = vec2 ((float)i,(float)j);
				additionalDims = vec2((float)potentialNewWidth, (float)potentialNewHeight);
			}
		}
}

vec2 HIGHOMEGA::Packer::pack(vec2 dims, vec3 color)
{
	vec2 emptySpaceStart, additionalDims;
	scanForEmpty((unsigned int)dims.x, (unsigned int)dims.y, emptySpaceStart, additionalDims);
	if (additionalDims.y > additionalDims.x) additionalDims.x = additionalDims.y;
	resize((unsigned int)additionalDims.x, (unsigned int)additionalDims.y);

	for (int x = (int)emptySpaceStart.x; x != (int)(emptySpaceStart.x + dims.x); x++)
		for (int y = (int)emptySpaceStart.y; y != (int)(emptySpaceStart.y + dims.y); y++)
		{
			int curIndex = (y * width) + x;
			cSpace[curIndex] = color;
		}

	return emptySpaceStart;
}

HIGHOMEGA::TexturePacker::TexturePacker()
{
	blockSize = 32;
	width = 0;
	height = 0;
	cSpace = nullptr;
}

void HIGHOMEGA::TexturePacker::resize(unsigned int newWidth, unsigned int newHeight)
{
	if (width == newWidth && height == newHeight) return;
	if (newWidth < width || newHeight < height) throw ("Both new width and height should be larger");

	unsigned char *newCSpace = (unsigned char *)malloc (newWidth * newHeight * 4);

	if (!newCSpace) throw ("Could not reallocate");

	for (int i = 0; i != newWidth * newHeight * 4; i++)
		newCSpace[i] = 0;

	for (int i = 0; i != height; i++)
		for (int j = 0; j != width; j++)
		{
			int curOldIndex = ((i * width) + j) * 4;
			int curNewIndex = ((i * newWidth) + j) * 4;
			newCSpace[curNewIndex] = cSpace[curOldIndex];
			newCSpace[curNewIndex + 1] = cSpace[curOldIndex + 1];
			newCSpace[curNewIndex + 2] = cSpace[curOldIndex + 2];
			newCSpace[curNewIndex + 3] = cSpace[curOldIndex + 3];
		}

	free((void *)cSpace);
	cSpace = newCSpace;
	width = newWidth;
	height = newHeight;
}

vec2 HIGHOMEGA::TexturePacker::pack(vec2 dims, unsigned int inpBpp, unsigned char *inpRawBuf)
{
	vec2 shrunkSize = vec2 (ceil (dims.x / ((float)blockSize)), ceil(dims.y / ((float)blockSize)));

	vec2 packLocation = objPacker.pack(shrunkSize, vec3((rand() % 100) * 0.01f, (rand() % 100) * 0.01f, (rand() % 100) * 0.01f));
	resize ((unsigned int)(objPacker.width * blockSize), (unsigned int)(objPacker.height * blockSize));
	vec2 retLocation = vec2(packLocation.x, packLocation.y) * (float)blockSize;

	for (int x = 0; x != (int)(dims.x); x++)
		for (int y =  0; y != (int)(dims.y); y++)
		{
			int ourIndex = (((int)(y + retLocation.y) * (int)width) + (int)(x + retLocation.x)) * 4;
			if (inpBpp == 4)
			{
				int imgIndex = ((y * (int)dims.x) + x) * 4;
				cSpace[ourIndex] = inpRawBuf[imgIndex];
				cSpace[ourIndex + 1] = inpRawBuf[imgIndex + 1];
				cSpace[ourIndex + 2] = inpRawBuf[imgIndex + 2];
				cSpace[ourIndex + 3] = inpRawBuf[imgIndex + 3];
			}
			else if (inpBpp == 1)
			{
				int imgIndex = ((y * (int)dims.x) + x);
				cSpace[ourIndex] = inpRawBuf[imgIndex];
				cSpace[ourIndex + 1] = inpRawBuf[imgIndex];
				cSpace[ourIndex + 2] = inpRawBuf[imgIndex];
				cSpace[ourIndex + 3] = (unsigned char)255;
			}
		}

	return retLocation;
}

unsigned char * HIGHOMEGA::TexturePacker::getCSpace()
{
	return cSpace;
}

unsigned int HIGHOMEGA::TexturePacker::getWidth()
{
	return width;
}

unsigned int HIGHOMEGA::TexturePacker::getHeight()
{
	return height;
}
