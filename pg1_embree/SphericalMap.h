#pragma once
#include "stdafx.h"
#include "vector3.h"
#include "texture.h"

class SphericalMap
{
public:
	SphericalMap(const std::string file_name);
	SphericalMap();
	Color3f texel(const float x, const float y, const float z) const;
private:
	std::unique_ptr<Texture> texture_;
};
