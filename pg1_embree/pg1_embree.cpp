#include "stdafx.h"
#include "tutorials.h"

int main()
{
	printf( "PG1, (c)2011-2020 Tomas Fabian\n\n" );

	_MM_SET_FLUSH_ZERO_MODE( _MM_FLUSH_ZERO_ON );
	_MM_SET_DENORMALS_ZERO_MODE( _MM_DENORMALS_ZERO_ON );

	//return tutorial_1();
	//return tutorial_2();
	return tutorial_3( "../../../data/6887_allied_avenger.obj" );
	//return tutorial_3("../../../data/geosphere.obj");
	//return tutorial_3("../../../data/geosphere_mirror.obj");
	//return tutorial_3("../../../data/geosphere_white_lambert.obj");
	//return tutorial_3("../../../data/geosphere_white_phong.obj");
	//return tutorial_3("../../../data/cornell_box2 - kopie.obj");
	//return tutorial_3("../../../data/cornell_box2.obj");
}
