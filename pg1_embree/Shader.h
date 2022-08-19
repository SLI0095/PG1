#pragma once
#include "stdafx.h"
#include "utils.h"
#include "camera.h"
#include "Light.h"
#include "texture.h"
#include "SphericalMap.h"
#include "mymath.h"
#include "material.h"
#include <random>

class Shader
{
public:
	Shader(Camera* camera, Light* light, SphericalMap* spherical_map, RTCScene scene, int sampling_rate, int shading_type);
	Shader();
	Color4f get_pixel(int x, int y);
private:
	Vector3 get_background(RTCRay dir);
	Vector3 diffuse_material();
	Vector3 glass_material();
	Vector3 traceRay(RTCRay ray, int depth);
	RTCRayHit setup_and_intersect_hit(RTCRay ray);
	RTCRay generate_shadow_ray(Vector3 ray_hit_point);
	RTCRay generate_reflection(Vector3 view_dir, Vector3 normal, Vector3 ray_hit_point, float ior);
	RTCRay generate_refraction(Vector3 dir, Vector3 normal, float last_ior, float material_hit_ior, Vector3 ray_hit_point);
	RTCRay generateSecondaryRay(Vector3 position, Vector3 direction, float ior);
	Vector3 generateReflectedVector(Vector3 vec, Vector3 normal);
	float c_srgb(float c_linear, float gamma = 2.4f);
	float c_linear(float c_srgb, float gamma = 2.4f);

	Camera* camera_;
	Light* light_;
	SphericalMap* spherical_map_;
	RTCScene scene_;
	
	int max_depth_;
	int sampling_rate_;
	int shading_type_;
	Vector3 color_;
};

