#pragma once
#include "stdafx.h"
#include "vector3.h"
class Light
{
public:
	Light(Vector3 position, Vector3 color);
	Light();
	Vector3 get_color();
	Vector3 get_positon();
private:
	Vector3 position_;
	Vector3 color_;
};

