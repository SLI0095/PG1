#include "stdafx.h"
#include "Light.h"

Light::Light(Vector3 position, Vector3 color)
{
	position_ = position;
	color_ = color;
}

Light::Light()
{
	color_ = Vector3(1.0f, 1.0f, 1.0f);
	position_ = Vector3(-115.0f, -95.0f, 110.0f);
}

Vector3 Light::get_color()
{
	return color_;
}

Vector3 Light::get_positon()
{
	return position_;
}
