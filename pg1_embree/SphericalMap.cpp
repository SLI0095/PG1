#include "stdafx.h"
#include "mymath.h"
#include "SphericalMap.h"

SphericalMap::SphericalMap(const std::string file_name)
{
	texture_ = std::unique_ptr<Texture>(new Texture(file_name.c_str()));
}

SphericalMap::SphericalMap()
{
	texture_ = std::unique_ptr<Texture>(new Texture("../../../data/ballroom_4k.hdr"));
}

Color3f SphericalMap::texel(const float x, const float y, const float z) const
{
	float theta, phi;

	theta = acosf(z);
	phi = atan2f(y, x) + M_PI;
	float u = 1.0f - phi * 0.5f * M_1_PI;
	float v = theta * M_1_PI;
	return texture_->get_texel(u,v);
}