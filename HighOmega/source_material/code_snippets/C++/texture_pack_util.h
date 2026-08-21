class Packer
{
	friend class TexturePacker;
private:
	unsigned int width, height;
	std::vector<vec3> cSpace;

	void resize(unsigned int newWidth, unsigned int newHeight);
	void scanForEmpty(unsigned int inpWidth, unsigned int inpHeight, vec2 & emptyStart, vec2 & additionalDims);

public:

	Packer();
	vec2 pack(vec2 dims, vec3 color);
};
class TexturePacker
{
private:
	unsigned int blockSize;
	unsigned int width, height;
	unsigned char *cSpace;
	Packer objPacker;

	void resize(unsigned int newWidth, unsigned int newHeight);

public:
	TexturePacker();
	vec2 pack(vec2 dims, unsigned int inpBpp, unsigned char *inpRawBuf);
	unsigned char *getCSpace();
	unsigned int getWidth();
	unsigned int getHeight();
};