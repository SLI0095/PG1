#include "stdafx.h"
#include "PathTracer.h"
#include "boost/math/special_functions/beta.hpp"
#include "boost/algorithm/clamp.hpp"
#include "utils.h"
using namespace boost::algorithm;
using namespace boost::math::detail;

PathTracer::PathTracer(Camera* camera, Light* light, SphericalMap* spherical_map, RTCScene scene, int number_of_samples, int max_depth)
{
	this->camera_ = camera;
	this->light_ = light;
	this->spherical_map_ = spherical_map;
	this->scene_ = scene;
	this->numer_of_samples_ = number_of_samples;
	this->max_depth_ = max_depth;
	this->furnace = false;
	this->energy_normalized = true;
}

PathTracer::PathTracer()
{
}

Color4f PathTracer::get_pixel(int x, int y)
{
	//printf("x = %d , y = %d \n", x, y);
	Vector3 pixel_color = Vector3(0.0f, 0.0f, 0.0f);
	for (int i = 0; i < numer_of_samples_; i++)
	{
		pixel_color += traceRay(camera_->GenerateRay(x, y), 0);
	}
	Vector3 final_color = Vector3(c_srgb(pixel_color.x / (float)numer_of_samples_), c_srgb(pixel_color.y / (float)numer_of_samples_), c_srgb(pixel_color.z / (float)numer_of_samples_));
	//Vector3 final_color = Vector3((pixel_color.x / numer_of_samples_), (pixel_color.y / numer_of_samples_), (pixel_color.z / numer_of_samples_));

	return Color4f{ final_color.x,final_color.y,final_color.z,1.0f };
}

Vector3 PathTracer::traceRay(RTCRay ray, int depth)
{
	if (depth > max_depth_)
	{
		if (furnace)
		{
			return Vector3(1, 1, 1);
		}
		return Vector3(0, 0, 0);
	}
		
	RTCRayHit ray_hit = setup_and_intersect_hit(ray);

	if (ray_hit.hit.geomID == RTC_INVALID_GEOMETRY_ID)
	{
		if (furnace)
		{
			return Vector3(1, 1, 1);
		}
		return get_background(ray_hit.ray);
	}

	RTCGeometry geometry = rtcGetGeometry(scene_, ray_hit.hit.geomID);
	Material* current_material = (Material*)(rtcGetGeometryUserData(geometry));

	float alpha = 1.0f;
	//Russian Rulette
	if (depth > 1)
	{
		alpha = min(max((current_material->diffuse.x), max((current_material->diffuse.y), (current_material->diffuse.z))), 0.95f);
		float random = Random(0.0f,1.0f);
		if (alpha <= random)
		{
			if (furnace)
			{
				return Vector3(1, 1, 1);
			}
			return Vector3(0, 0, 0);
		}
	}
	

	Vector3 normal;
	rtcInterpolate0(geometry, ray_hit.hit.primID, ray_hit.hit.u, ray_hit.hit.v,
		RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, &normal.x, 3);
	normal.Normalize();

	Vector3 ray_hit_point = Vector3(ray_hit.ray.org_x + ray_hit.ray.dir_x * ray_hit.ray.tfar, ray_hit.ray.org_y + ray_hit.ray.dir_y * ray_hit.ray.tfar, ray_hit.ray.org_z + ray_hit.ray.dir_z * ray_hit.ray.tfar);

	Vector3 dir = Vector3(ray_hit.ray.dir_x, ray_hit.ray.dir_y, ray_hit.ray.dir_z);
	dir.Normalize();

	Vector3 dir_to_cam = dir * -1.0f;
	dir_to_cam.Normalize();
	Vector3 omega_o = dir_to_cam;

	if (normal.DotProduct(dir) > 0.0f)
	{
		normal *= -1;
		normal.Normalize();
	}

	float last_ior = ray_hit.ray.time;
	float material_hit_ior = current_material->ior;

	Vector3 L_e = Vector3(current_material->emission.x, current_material->emission.y, current_material->emission.z);

	if (L_e.x != 0.0f || L_e.y != 0.0f || L_e.z != 0.0f)
	{
		//L_e = Vector3(c_linear(L_e.x), c_linear(L_e.y), c_linear(L_e.z));
		L_e = Vector3((L_e.x), (L_e.y), (L_e.z));

		return L_e;
	}
	if (current_material->shader_number == 6) //mirror
	{
		float pdf;
		Vector3 omega_i;
		omega_i = generateReflectedVector(dir_to_cam, normal);
		return traceRay(generateSecondaryRay(ray_hit_point, omega_i, last_ior), depth + 1);
	}
	else if (current_material->shader_number == 5) //Lambert
	{
		float pdf;
		Vector3 omega_i;
		hemisphereSampling(normal, omega_i, pdf);
		//hemisphereCosineWeightedSampling(normal, omega_i, pdf);
		//Vector3 albedo = Vector3(c_linear(current_material->diffuse.x), c_linear(current_material->diffuse.y), c_linear(current_material->diffuse.z));
		Vector3 diffuse = Vector3((current_material->diffuse.x), (current_material->diffuse.y), (current_material->diffuse.z));
		Vector3 f_r = diffuse * Vector3(1 / M_PI, 1 / M_PI, 1 / M_PI);
		Vector3 L_i = traceRay(generateSecondaryRay(ray_hit_point, omega_i, last_ior), depth + 1);
		//L_i = Vector3(c_linear(L_i.x), c_linear(L_i.y), c_linear(L_i.z));
		Vector3 L_r = L_i * f_r * omega_i.DotProduct(normal) / (pdf * alpha);
		//L_r = Vector3(c_linear(L_r.x), c_linear(L_r.y), c_linear(L_r.z));

		return L_r;
	}
	else if (current_material->shader_number == 7) //Phong
	{
		Vector3 diffuse = Vector3((current_material->diffuse.x), (current_material->diffuse.y), (current_material->diffuse.z));
		Vector3 specular = Vector3((current_material->specular.x), (current_material->specular.y), (current_material->specular.z));
		float shinnes = current_material->shininess;

		float pdf;
		Vector3 omega_i;
		Vector3 omega_r = generateReflectedVector(dir_to_cam, normal);
		omega_r.Normalize();


		float max_diff = max(diffuse.x, max(diffuse.y, diffuse.z));
		float max_spec = max(specular.x, max(specular.y, specular.z));
		float random = Random(0.0f, max_diff + max_spec);
		if (random < max_diff)
		{
			hemisphereCosineWeightedSampling(normal, omega_i, pdf);
			float one_over_Im = (shinnes + 2.0f) / (2.0f * M_PI);
			Vector3 F = fresnel(specular, omega_o.DotProduct(normal));
			Vector3 Rd = ((1.0f - max(F.x, max(F.y, F.z))) / (1.0f - max(specular.x, max(specular.y, specular.z)))) * diffuse;
			Vector3 ks = F * one_over_Im * pow(omega_i.DotProduct(omega_r), shinnes);
			Vector3 kd = Rd / M_PI;
			Vector3 f_r = kd /*+ ks*/;
			Vector3 L_i = traceRay(generateSecondaryRay(ray_hit_point, omega_i, last_ior), depth + 1);
			Vector3 L_r = L_i * f_r * omega_i.DotProduct(normal) / (pdf * alpha * (1.0f - random));
			return L_r;
		}
		else
		{
			if (!energy_normalized) //energy - conserving
			{
				cosineLobeSampling(omega_r, shinnes, omega_i, pdf);
				float one_over_Im = (shinnes + 2.0f) / (2.0f * M_PI);

				Vector3 F = fresnel(specular, omega_o.DotProduct(normal));
				Vector3 Rd = ((1.0f - max(F.x, max(F.y, F.z))) / (1.0f - max(specular.x, max(specular.y, specular.z)))) * diffuse;
				Vector3 ks = F * one_over_Im * (pow(omega_i.DotProduct(omega_r), shinnes) / omega_i.DotProduct(normal));
				Vector3 kd = Rd / M_PI;
				Vector3 f_r = /*kd +*/ ks;
				Vector3 L_i = traceRay(generateSecondaryRay(ray_hit_point, omega_i, last_ior), depth + 1);
				Vector3 L_r = L_i * f_r * omega_i.DotProduct(normal) / (pdf * alpha * (random));
				return L_r;
			}
			else //energy - normalizing
			{
				cosineLobeSampling(omega_r, shinnes, omega_i, pdf);
				float Im = calc_I_m(clamp(normal.DotProduct(omega_i), 0.0f, 1.0f), shinnes);
				float one_over_Im = 1.0f / Im;

				Vector3 F = fresnel(specular, omega_o.DotProduct(normal));
				Vector3 Rd = ((1.0f - max(F.x, max(F.y, F.z))) / (1.0f - max(specular.x, max(specular.y, specular.z)))) * diffuse;
				Vector3 ks = F * one_over_Im * (pow(omega_i.DotProduct(omega_r), shinnes) /*/ omega_i.DotProduct(normal)*/);
				Vector3 kd = Rd / M_PI;
				Vector3 f_r = /*kd +*/ ks;
				Vector3 L_i = traceRay(generateSecondaryRay(ray_hit_point, omega_i, last_ior), depth + 1);
				Vector3 L_r = L_i * f_r * omega_i.DotProduct(normal) / (pdf * alpha * (random));
				return L_r;
				/*float Im = calc_I_m();
				float one_over_Im = 1.0f / Im;*/
			}
		}
		

	}
	else if (current_material->shader_number == 4) //Glass
	{
		//float n1 = last_ior;
		//float n2 = material_hit_ior;

		//if (n1 != IOR_AIR)
		//{
		//	n2 = IOR_AIR;
		//}
		//float n1_over_n2 = n1 / n2;
		//float Q1 = normal.DotProduct(dir_to_cam);

		//Vector3 omega_i;
		//omega_i = generateReflectedVector(dir_to_cam, normal);

		////total reflection
		//const float tmp = 1.f - sqrt(n1_over_n2) * (1.f - sqrt(Q1));
		//if (tmp < 0.f) 
		//{
		//	return traceRay(generateSecondaryRay(ray_hit_point, omega_i, n1), depth + 1);
		//}

		//const float Q2 = sqrt(tmp);
		//Vector3 refract_dir;

		//refract_dir = (n1_over_n2 * dir) + ((n1_over_n2 * Q1 - Q2) * normal);
		//refract_dir.Normalize();

		//float Rs = sqr((n1 * Q2 - n2 * Q1) / (n1 * Q2 + n2 * Q1));
		//float Rp = sqr((n1 * Q1 - n2 * Q2) / (n1 * Q1 + n2 * Q2));

		//float R = 0.5f * (Rs + Rp);

		//Vector3 diffuse = current_material->diffuse;
		//Vector3 reflected_part = traceRay(generateSecondaryRay(ray_hit_point, omega_i, n1), depth + 1) * R;
		//Vector3 refracted_part = traceRay(generateSecondaryRay(ray_hit_point, refract_dir, n2), depth + 1) * (1.0f - R) * diffuse;
		//Vector3 L_r = reflected_part + refracted_part;
		////L_r = Vector3(c_linear(L_r.x), c_linear(L_r.y), c_linear(L_r.z));
		//return L_r;




		float n1 = last_ior;
		float n2 = material_hit_ior;

		if (n1 != IOR_AIR)
		{
			n2 = IOR_AIR;
		}
		float n1_over_n2 = n1 / n2;
		float under_sqrt = 1 - sqr(n1_over_n2) * (1 - sqr(dir.DotProduct(normal)));

		Vector3 reflection;
		reflection = generateReflectedVector(dir_to_cam, normal);

		//total reflection
		if (under_sqrt < 0.0f)
		{
			return traceRay(generateSecondaryRay(ray_hit_point, reflection, n1), depth + 1);
		}

		Vector3 refract_dir;

		refract_dir = (n1_over_n2 * dir) - (n1_over_n2 * dir.DotProduct(normal) + sqrt(under_sqrt)) * normal;
		refract_dir.Normalize();

		float cos_theta1 = dir_to_cam.DotProduct(normal);
		float cos_theta2 = sqrt(1 - sqr(n1_over_n2) * (1 - sqr(cos_theta1)));

		float Rs = sqr((n2 * cos_theta2 - n1 * cos_theta1) / (n2 * cos_theta2 + n1 * cos_theta1));
		float Rp = sqr((n2 * cos_theta1 - n1 * cos_theta2) / (n2 * cos_theta1 + n1 * cos_theta2));

		float R = 0.5f * (Rs + Rp);

		Vector3 diffuse = current_material->diffuse;
		//diffuse = Vector3(c_linear(diffuse.x), c_linear(diffuse.y), c_linear(diffuse.z));
		Vector3 reflected_part = traceRay(generateSecondaryRay(ray_hit_point, reflection, n1), depth + 1) * R;
		Vector3 refracted_part = traceRay(generateSecondaryRay(ray_hit_point, refract_dir, n2), depth + 1) * (1.0f - R) * diffuse;
		Vector3 L_r = reflected_part + refracted_part;
		return L_r;

		//float R_0 = pow(((n1 - n2) / (n1 + n2)), 2);
		//float R = R_0 + (1 - R_0) * pow((1 - normal.DotProduct(dir_to_cam)), 5);

		//Vector3 omega_i;
		//omega_i = generateReflectedVector(dir_to_cam, normal);

		//float n1_over_n2 = n1 / n2;

		//float under_sqrt = 1.0f - pow(n1_over_n2, 2) * (1.0f - pow(dir.DotProduct(normal), 2));
		//if (under_sqrt < 0)
		//{
		//	return traceRay(generateSecondaryRay(ray_hit_point, omega_i, last_ior), depth + 1);
		//}
		//else
		//{
		//	Vector3 refract_dir;
		//	refract_dir = n1_over_n2 * dir - (n1_over_n2 * dir_to_cam.DotProduct(dir) + sqrt(under_sqrt)) * normal;
		//	refract_dir.Normalize();
		//	Vector3 reflected_part = traceRay(generateSecondaryRay(ray_hit_point, omega_i, last_ior), depth + 1) * R;
		//	Vector3 refracted_part = traceRay(generateSecondaryRay(ray_hit_point, refract_dir, material_hit_ior), depth + 1) * (1.0f - R);
		//	Vector3 diffuse = current_material->diffuse;
		//	return reflected_part + refracted_part;
		//	//return (traceRay(generateSecondaryRay(ray_hit_point,refract_dir,material_hit_ior), depth + 1) * (1.0f - R)) + (traceRay(generateSecondaryRay(ray_hit_point, omega_i, last_ior), depth + 1 ) * R);
		//}

		/*refract_dir = n1_over_n2 * dir - (n1_over_n2 * dir.DotProduct(normal) + sqrtf(1.0f - pow(n1_over_n2, 2) * (1.0f - pow(clamp(dir.DotProduct(normal),0.0f,1.0f), 2)))) * normal;
		refract_dir.Normalize();*/



		//Vector3 diffuse = current_material->diffuse;
		//return (traceRay(generate_reflection(view_dir, normal, ray_hit_point, ray_hit.ray.time), depth + 1) * R) + (/*diffuse * */traceRay(generate_refraction(dir, normal, ray_hit.ray.time, current_material->ior, ray_hit_point), depth + 1) * (1.0f - R));
	}
}

RTCRayHit PathTracer::setup_and_intersect_hit(RTCRay ray)
{
	RTCHit hit;
	hit.geomID = RTC_INVALID_GEOMETRY_ID;
	hit.primID = RTC_INVALID_GEOMETRY_ID;
	hit.Ng_x = 0.0f; // geometry normal
	hit.Ng_y = 0.0f;
	hit.Ng_z = 0.0f;

	// merge ray and hit structures
	RTCRayHit ray_hit;
	ray_hit.ray = ray;
	ray_hit.hit = hit;

	// intersect ray with the scene
	RTCIntersectContext context;
	rtcInitIntersectContext(&context);
	rtcIntersect1(scene_, &context, &ray_hit);
	return ray_hit;
}

RTCRay PathTracer::generate_reflection(Vector3 view_dir, Vector3 normal, Vector3 ray_hit_point, float ior)
{
	Vector3 dir = 2.0f * (clamp(view_dir.DotProduct(normal),0.0f,1.0f)) * normal - view_dir;
	dir.Normalize();
	//dir = fromLocalToWorld(dir, normal);
	//dir.Normalize();
	RTCRay reflected_ray = RTCRay();

	reflected_ray.org_x = ray_hit_point.x; // ray origin
	reflected_ray.org_y = ray_hit_point.y;
	reflected_ray.org_z = ray_hit_point.z;
	reflected_ray.tnear = 0.01f; // start of ray segment

	reflected_ray.dir_x = dir.x; // ray direction
	reflected_ray.dir_y = dir.y;
	reflected_ray.dir_z = dir.z;
	reflected_ray.time = ior; // time of this ray for motion blur

	reflected_ray.tfar = FLT_MAX; // end of ray segment (set to hit distance)

	reflected_ray.mask = 0; // can be used to mask out some geometries for some rays
	reflected_ray.id = 0; // identify a ray inside a callback function
	reflected_ray.flags = 0; // reserved

	return reflected_ray;
}

RTCRay PathTracer::generate_refraction(Vector3 dir, Vector3 normal, float last_ior, float material_hit_ior, Vector3 ray_hit_point)
{
	Vector3 refract_dir;

	float n1 = last_ior;
	float n2 = material_hit_ior;
	if (n1 != IOR_AIR)
	{
		n2 = IOR_AIR;
	}


	float n1_over_n2 = n1 / n2;

	refract_dir = n1_over_n2 * dir - (n1_over_n2 * dir.DotProduct(normal) + sqrtf(1.0f - pow(n1_over_n2, 2) * (1.0f - pow(dir.DotProduct(normal), 2)))) * normal;
	refract_dir.Normalize();

	RTCRay refracted_ray = RTCRay();

	refracted_ray.org_x = ray_hit_point.x; // ray origin
	refracted_ray.org_y = ray_hit_point.y;
	refracted_ray.org_z = ray_hit_point.z;
	refracted_ray.tnear = 0.01f; // start of ray segment

	refracted_ray.dir_x = refract_dir.x; // ray direction
	refracted_ray.dir_y = refract_dir.y;
	refracted_ray.dir_z = refract_dir.z;
	refracted_ray.time = material_hit_ior; // time of this ray for motion blur

	refracted_ray.tfar = FLT_MAX; // end of ray segment (set to hit distance)

	refracted_ray.mask = 0; // can be used to mask out some geometries for some rays
	refracted_ray.id = 0; // identify a ray inside a callback function
	refracted_ray.flags = 0; // reserved

	return refracted_ray;
}

Vector3 PathTracer::get_background(RTCRay dir)
{
	if (spherical_map_ == NULL)
	{
		return Vector3(0.0f, 0.0f, 0.0f);
	}
	Color3f bckg = spherical_map_->texel(dir.dir_x, dir.dir_y, dir.dir_z);
	return Vector3(c_linear(bckg.r), c_linear(bckg.g), c_linear(bckg.b));
}

void PathTracer::hemisphereSampling(Vector3 normal, Vector3& omega_i, float& pdf)
{
	const float Pie_2 = 2.0f * M_PI;
	float random1;
	float random2;
	do
	{
		random1 = Random(0.0f, 1.0f);
		random2 = Random(0.0f, 1.0f);

	} while (random1 >= 1.0f || random2 >= 1.0f);

	float x = 2.0f * cos(Pie_2 * random1) * sqrt(random2 * (1.0f - random2));
	float y = 2.0f * sin(Pie_2 * random1) * sqrt(random2 * (1.0f - random2));
	float z = 1.0f - 2.0f * random2;

	pdf = 1.0f / Pie_2;

	omega_i = Vector3(x, y, z);
	omega_i.Normalize();
	
	if (omega_i.DotProduct(normal) < 0) 
	{
		omega_i *= -1.0f;
	}
}

void PathTracer::hemisphereCosineWeightedSampling(Vector3 normal, Vector3& omega_i, float& pdf)
{
	const float Pie_2 = 2.0f * M_PI;
	float random1;
	float random2;
	do
	{
		random1 = Random(0.0f, 1.0f);
		random2 = Random(0.0f, 1.0f);

	} while (random1 >= 1.0f || random2 >= 1.0f);

	float x = cos(Pie_2 * random1) * sqrt(1.0f - random2);
	float y = sin(Pie_2 * random1) * sqrt(1.0f - random2);
	float z = sqrt(random2);
	
	omega_i = Vector3(x, y, z);
	omega_i = fromLocalToWorld(omega_i, normal);
	omega_i.Normalize();
	pdf = omega_i.DotProduct(normal) / M_PI;

	//float theta = acos(z);
	//pdf = cos(theta) / M_PI;
}

void PathTracer::cosineLobeSampling(Vector3 omega_r, float shiness, Vector3& omega_i, float& pdf)
{
	const float Pie_2 = 2.0f * M_PI;
	float random1;
	float random2;
	do
	{
		random1 = Random(0.0f, 1.0f);
		random2 = Random(0.0f, 1.0f);

	} while (random1 >= 1.0f || random2 >= 1.0f);
	
	float exponent1 = (1.0f) / (shiness + 1.0f);
	float exponent2 = (2.0f) / (shiness + 1.0f);

	float z = pow(random2, exponent1);
	float x = cos(Pie_2 * random1) * sqrt(1.0f - pow(random2,exponent2));
	float y = sin(Pie_2 * random1) * sqrt(1.0f - pow(random2, exponent2));
	
	//pdf = ((shiness + 1.0f)/Pie_2) * pow(z,shiness);

	omega_i = Vector3(x, y, z);
	omega_i.Normalize();

	omega_i = fromLocalToWorld(omega_i, omega_r);
	omega_i.Normalize();

	pdf = ((shiness + 1.0f) / Pie_2) * pow(z, shiness);
}

RTCRay PathTracer::generateSecondaryRay(Vector3 position, Vector3 direction, float ior)
{
	RTCRay ray = RTCRay();

	ray.org_x = position.x; // ray origin
	ray.org_y = position.y;
	ray.org_z = position.z;
	ray.tnear = 0.01f; // start of ray segment

	ray.dir_x = direction.x; // ray direction
	ray.dir_y = direction.y;
	ray.dir_z = direction.z;
	ray.time = ior; // time of this ray for motion blur

	ray.tfar = FLT_MAX; // end of ray segment (set to hit distance)

	ray.mask = 0; // can be used to mask out some geometries for some rays
	ray.id = 0; // identify a ray inside a callback function
	ray.flags = 0; // reserved

	return ray;
}

Vector3 PathTracer::fromLocalToWorld(Vector3 vec, Vector3 normal)
{
	Vector3 a = (abs(normal.x) > abs(normal.z)) ? Vector3( normal.y, -normal.x, 0.0f) : Vector3(0.0f, normal.z, -normal.y);
	a.Normalize();

	Vector3 o2 = normal.CrossProduct(a);
	o2.Normalize();
	Vector3 o1 = o2.CrossProduct(normal);
	o1.Normalize();

	Matrix3x3 transformMat = Matrix3x3(o1, o2, normal);
	return transformMat * vec;
}

Vector3 PathTracer::generateReflectedVector(Vector3 vec, Vector3 normal)
{
	Vector3 reflect = 2.0f * (clamp(vec.DotProduct(normal), 0.0f, 1.0f)) * normal - vec;
	reflect.Normalize();
	return reflect;
}

Vector3 PathTracer::fresnel(Vector3 F_0, float cos_theta)
{
	return F_0 + (Vector3(1.0f, 1.0f, 1.0f) - F_0) * pow(1.0f- cos_theta, 5);
}

float PathTracer::ibeta(float x, float a, float b) {
	return boost::math::beta(a, b, x);
}

float PathTracer::gamma_quot(float a, float b)
{
	return exp(lgamma(a) - lgamma(b));
}

float PathTracer::calc_I_m(float N_dot_omega_i, float shinnes)
{
	float Pie_2 = 2.0f * M_PI;
	float sqrtPie = sqrt(M_PI);

	float costerm = N_dot_omega_i;
	float sinterm_sq = 1.0f - costerm * costerm;
	float halfn = 0.5f * shinnes;

	float negterm = costerm;
	if (shinnes > 1e-18f)
	{
		negterm *= halfn * ibeta(sinterm_sq, halfn, 0.5f);
	}

	return (
		Pie_2 * costerm +
		sqrtPie * gamma_quot(halfn + 0.5f, halfn + 1.0f) *
		(pow(sinterm_sq, halfn) - negterm)
		) / (shinnes + 2.0f);
}


float PathTracer::c_srgb(float c_linear, float gamma)
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

float PathTracer::c_linear(float c_srgb, float gamma)
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
