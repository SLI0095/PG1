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

class PathTracer
{
public:
	PathTracer(Camera* camera, Light* light, SphericalMap* spherical_map, RTCScene scene, int number_of_samples, int max_depth);
	PathTracer();
	Color4f get_pixel(int x, int y);
private:
	Vector3 traceRay(RTCRay ray, int depth);

	RTCRayHit setup_and_intersect_hit(RTCRay ray);
	RTCRay generate_reflection(Vector3 view_dir, Vector3 normal, Vector3 ray_hit_point, float ior);
	RTCRay generate_refraction(Vector3 dir, Vector3 normal, float last_ior, float material_hit_ior, Vector3 ray_hit_point);
	RTCRay generateSecondaryRay(Vector3 position, Vector3 direction, float ior);
	Vector3 fromLocalToWorld(Vector3 vec, Vector3 normal);
	Vector3 generateReflectedVector(Vector3 vec, Vector3 normal);
	Vector3 fresnel(Vector3 F_0, float cos);

	float ibeta(float x, float a, float b);
	float gamma_quot(float a, float b);
	float calc_I_m(float N_dot_omega_i, float shinnes);

	Vector3 get_background(RTCRay dir);
	void hemisphereSampling(Vector3 normal, Vector3& omega_i, float& pdf);
	void hemisphereCosineWeightedSampling(Vector3 normal, Vector3& omega_i, float& pdf);
	void hemisphereCosineWeightedSampling(Vector3 normal, Vector3 omega_o, Vector3& omega_i, float& pdf);
	void cosineLobeSampling(Vector3 omega_r, float shiness, Vector3& omega_i, float& pdf);

	float c_srgb(float c_linear, float gamma = 2.4f);
	float c_linear(float c_srgb, float gamma = 2.4f);


	Camera* camera_;
	Light* light_;
	SphericalMap* spherical_map_;
	RTCScene scene_;
	
	int numer_of_samples_;
	int max_depth_;
	bool furnace;
	bool energy_normalized;
};


