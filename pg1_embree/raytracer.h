#pragma once
#include "simpleguidx11.h"
#include "surface.h"
#include "camera.h"
#include "SphericalMap.h"
#include "Shader.h"
#include "PathTracer.h"

/*! \class Raytracer
\brief General ray tracer class.

\author Tomáš Fabián
\version 0.1
\date 2018
*/
class Raytracer : public SimpleGuiDX11
{
public:
	Raytracer( const int width, const int height, 
		const float fov_y, const Vector3 view_from, const Vector3 view_at,
		const char * config = "threads=0,verbose=3" );
	~Raytracer();

	int InitDeviceAndScene( const char * config );

	int ReleaseDeviceAndScene();

	void LoadScene( const std::string file_name );

	RTCRay GenerateShadowRay(Vector3 light_pos, Vector3 ray_from);

	float c_srgb(float c_linear, float gamma = 2.4f);
	float c_linear(float c_srgb, float gamma = 2.4f);

	Color4f get_pixel( const int x, const int y, const float t = 0.0f ) override;

	int Ui();

private:
	std::vector<Surface *> surfaces_;
	std::vector<Material *> materials_;

	RTCDevice device_;
	RTCScene scene_;
	Camera camera_;
	SphericalMap spherical_map_;
	Shader shader_;
	Vector3 light_pos_;
	PathTracer pathTracer_;

	int pixel_count_;
	long total_pixels_;
};
