...

TextureAtlasClass HIGHOMEGA::RENDER::textureAtlas;

...


HIGHOMEGA::RENDER::TextureAtlasClass::TextureAtlasClass()
{
	created = false;
}

void HIGHOMEGA::RENDER::TextureAtlasClass::Create()
{
	if (created) return;

	atlasImage.CreateTexture(Instance, textureAtlas.packer.getWidth(), textureAtlas.packer.getHeight(), textureAtlas.packer.getCSpace(), 1, false, false, false);

	atlasDims.invAtlasWidth = 1.0f/((float)textureAtlas.packer.getWidth());
	atlasDims.invAtlasHeight = 1.0f/((float)textureAtlas.packer.getHeight());
	new (&dimsBuf) BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &atlasDims, sizeof(atlasDimsStruct));

	created = true;
}


CacheItem<ImageClass> * HIGHOMEGA::RENDER::AddOrFindCachedTexture(std::string & belong, std::string & texName, InstanceClass & ptrToInstance, bool isArray, int nLayers, bool mipmap)
{
		...
		try
		{
			if (isArray)
				TextureCache[texName].elem.CreateTexture(ptrToInstance, belong + texName, nLayers, false, true, false, mipmap);
			else
				TextureCache[texName].elem.CreateTexture (ptrToInstance, belong + texName, 1U, false, false, false, mipmap);
			ImageClass & curElem = TextureCache[texName].elem;
			--> curElem.textureAtlasCoords = textureAtlas.packer.pack(vec2((float)curElem.getWidth(), (float)curElem.getHeight()), curElem.getBPP(), (unsigned char *)curElem.getData());
			TextureCache[texName].elemCount = 1;
		}
		...
}