class TextureAtlasClass
{
private:
	bool created;
public:
	struct atlasDimsStruct
	{
		float invAtlasHeight;
		float invAtlasWidth;
		float reserved[2];
	} atlasDims;
	BufferClass dimsBuf;
	ImageClass atlasImage;
	TexturePacker packer;

	TextureAtlasClass();
	void Create();
};

extern TextureAtlasClass textureAtlas;