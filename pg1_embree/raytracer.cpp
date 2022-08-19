#include "stdafx.h"
#include "raytracer.h"
#include "objloader.h"
#include "tutorials.h"
#include "SphericalMap.h"
Raytracer::Raytracer(const int width, const int height,
	const float fov_y, const Vector3 view_from, const Vector3 view_at,
	const char* config) : SimpleGuiDX11(width, height)
{
	InitDeviceAndScene(config);

	shader_ = Shader(new Camera(width, height, fov_y, view_from, view_at), new Light(Vector3(-115.0f, -95.0f, 110.0f), Vector3(1.0f, 1.0f, 1.0f)), new SphericalMap("../../../data/ballroom.jpg"), scene_, 1000,4);
	//pathTracer_ = PathTracer(new Camera(width, height, fov_y, view_from, view_at), new Light(Vector3(0.0f, 0.0f, 485.0f), Vector3(1.0f, 1.0f, 1.0f)), NULL, scene_, 10000, 50);
	//pathTracer_ = PathTracer(new Camera(width, height, fov_y, view_from, view_at), new Light(Vector3(0.0f, 0.0f, 485.0f), Vector3(1.0f, 1.0f, 1.0f)), new SphericalMap("../../../data/ballroom.jpg"), scene_, 2000, 20);
	/*spherical_map_ = SphericalMap("../../../data/ballroom.jpg");
	camera_ = Camera(width, height, fov_y, view_from, view_at);
	light_pos_ = Vector3(-115.0f, -95.0f, 110.0f);*/
	pixel_count_ = 0;
	total_pixels_ = width * height;
}

Raytracer::~Raytracer()
{
	ReleaseDeviceAndScene();
}

int Raytracer::InitDeviceAndScene( const char * config )
{
	device_ = rtcNewDevice( config );
	error_handler( nullptr, rtcGetDeviceError( device_ ), "Unable to create a new device.\n" );
	rtcSetDeviceErrorFunction( device_, error_handler, nullptr );

	ssize_t triangle_supported = rtcGetDeviceProperty( device_, RTC_DEVICE_PROPERTY_TRIANGLE_GEOMETRY_SUPPORTED );

	// create a new scene bound to the specified device
	scene_ = rtcNewScene( device_ );

	return S_OK;
}

int Raytracer::ReleaseDeviceAndScene()
{
	rtcReleaseScene( scene_ );
	rtcReleaseDevice( device_ );

	return S_OK;
}

void Raytracer::LoadScene( const std::string file_name )
{
	const int no_surfaces = LoadOBJ( file_name.c_str(), surfaces_, materials_ );

	// surfaces loop
	for ( auto surface : surfaces_ )
	{
		RTCGeometry mesh = rtcNewGeometry( device_, RTC_GEOMETRY_TYPE_TRIANGLE );

		Vertex3f * vertices = ( Vertex3f * )rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
			sizeof( Vertex3f ), 3 * surface->no_triangles() );

		Triangle3ui * triangles = ( Triangle3ui * )rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
			sizeof( Triangle3ui ), surface->no_triangles() );

		rtcSetGeometryUserData( mesh, ( void* )( surface->get_material() ) );

		rtcSetGeometryVertexAttributeCount( mesh, 2 );

		Normal3f * normals = ( Normal3f * )rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, RTC_FORMAT_FLOAT3,
			sizeof( Normal3f ), 3 * surface->no_triangles() );

		Coord2f * tex_coords = ( Coord2f * )rtcSetNewGeometryBuffer(
			mesh, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, RTC_FORMAT_FLOAT2,
			sizeof( Coord2f ), 3 * surface->no_triangles() );		

		// triangles loop
		for ( int i = 0, k = 0; i < surface->no_triangles(); ++i )
		{
			Triangle & triangle = surface->get_triangle( i );

			// vertices loop
			for ( int j = 0; j < 3; ++j, ++k )
			{
				const Vertex & vertex = triangle.vertex( j );

				vertices[k].x = vertex.position.x;
				vertices[k].y = vertex.position.y;
				vertices[k].z = vertex.position.z;

				normals[k].x = vertex.normal.x;
				normals[k].y = vertex.normal.y;
				normals[k].z = vertex.normal.z;

				tex_coords[k].u = vertex.texture_coords[0].u;
				tex_coords[k].v = vertex.texture_coords[0].v;
			} // end of vertices loop

			triangles[i].v0 = k - 3;
			triangles[i].v1 = k - 2;
			triangles[i].v2 = k - 1;
		} // end of triangles loop

		rtcCommitGeometry( mesh );
		unsigned int geom_id = rtcAttachGeometry( scene_, mesh );
		rtcReleaseGeometry( mesh );
	} // end of surfaces loop

	rtcCommitScene( scene_ );
}

RTCRay Raytracer::GenerateShadowRay(Vector3 light_pos, Vector3 ray_from)
{
	RTCRay shadow_ray = RTCRay();

	shadow_ray.org_x = ray_from.x; // ray origin
	shadow_ray.org_y = ray_from.y;
	shadow_ray.org_z = ray_from.z;
	shadow_ray.tnear = FLT_MIN; // start of ray segment

	shadow_ray.dir_x = light_pos.x - ray_from.x; // ray direction
	shadow_ray.dir_y = light_pos.y - ray_from.y;
	shadow_ray.dir_z = light_pos.z - ray_from.z;
	shadow_ray.time = 0.0f; // time of this ray for motion blur

	shadow_ray.tfar = FLT_MAX; // end of ray segment (set to hit distance)

	shadow_ray.mask = 0; // can be used to mask out some geometries for some rays
	shadow_ray.id = 0; // identify a ray inside a callback function
	shadow_ray.flags = 0; // reserved

	return shadow_ray;
}

float Raytracer::c_srgb(float c_linear, float gamma)
{
	if (c_linear <= 0.0f) return 0.0f;
	else if (c_linear >= 1.0f) return 1.0f;
	assert((c_linear >= 0.0f) && (c_linear <= 1.0f));
	if (c_linear <= 0.0031308f)
	{
		return 12.92f * c_linear;
	}
	else
	{
		const float a = 0.055f;
		return (1.0f + a) * powf(c_linear, 1.0f / gamma) - a;
	}

}

float Raytracer::c_linear(float c_srgb, float gamma)
{
	if (c_srgb <= 0.0f) return 0.0f;
	else if (c_srgb >= 1.0f) return 1.0f;
	assert((c_srgb >= 0.0f) && (c_srgb <= 1.0f));
	if (c_srgb <= 0.04045f)
	{
		return c_srgb / 12.92f;
	}
	else
	{
		const float a = 0.055f;
		return powf((c_srgb + a) / (1.0f + a), gamma);
	}
}

Color4f Raytracer::get_pixel( const int x, const int y, const float t )
{
	//RayTracer

	Color4f result = shader_.get_pixel(x, y);
	return result;


	//PathTracer
	/*Color4f result = pathTracer_.get_pixel(x, y);
	return result;*/
}

int Raytracer::Ui()
{
	static float f = 0.0f;
	static int counter = 0;

	// Use a Begin/End pair to created a named window
	ImGui::Begin( "Ray Tracer Params" );
	
	ImGui::Text( "Surfaces = %d", surfaces_.size() );
	ImGui::Text( "Materials = %d", materials_.size() );
	ImGui::Separator();
	ImGui::Checkbox( "Vsync", &vsync_ );
	
	//ImGui::Checkbox( "Demo Window", &show_demo_window ); // Edit bools storing our window open/close state
	//ImGui::Checkbox( "Another Window", &show_another_window );

	ImGui::SliderFloat( "float", &f, 0.0f, 1.0f ); // Edit 1 float using a slider from 0.0f to 1.0f    
	//ImGui::ColorEdit3( "clear color", ( float* )&clear_color ); // Edit 3 floats representing a color

	// Buttons return true when clicked (most widgets return true when edited/activated)
	if ( ImGui::Button( "Button" ) )
		counter++;
	ImGui::SameLine();
	ImGui::Text( "counter = %d", counter );

	ImGui::Text( "Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate );
	ImGui::End();

	// 3. Show another simple window.
	/*if ( show_another_window )
	{
	ImGui::Begin( "Another Window", &show_another_window ); // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
	ImGui::Text( "Hello from another window!" );
	if ( ImGui::Button( "Close Me" ) )
	show_another_window = false;
	ImGui::End();
	}*/

	return 0;
}
