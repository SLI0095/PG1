#include "stdafx.h"
#include "Shader.h"
#include "boost/algorithm/clamp.hpp"
using namespace boost::algorithm;

Shader::Shader(Camera* camera, Light* light, SphericalMap* spherical_map, RTCScene scene, int sampling_rate, int shading_type)
{
	camera_ = camera;
	light_ = light;
	spherical_map_ = spherical_map;
	scene_ = scene;
	max_depth_ = 5;
	sampling_rate_ = sampling_rate;
	/*if (sampling_rate_ % 10 != 0)
	{
		sampling_rate_ *= 10;
	}*/
	shading_type_ = shading_type;
}

Shader::Shader()
{
}

Color4f Shader::get_pixel(int x, int y)
{
	Vector3 color = Vector3(0.0f, 0.0f, 0.0f);

	Vector3 generateRandomVector();

	for (int i = 0; i < sampling_rate_; i++)
	{
		float ksi_x = Random(0.0f, 1.0f);
		float ksi_y = Random(0.0f, 1.0f);
		color += traceRay(camera_->GenerateDepthOfField((float)x + ksi_x, (float)y + ksi_y), 0);
		//color += traceRay(camera_->GenerateRay((float)x + ksi_x, (float)y + ksi_y), 0);
	}
		
	Vector3 final_color = Vector3(color.x / (sampling_rate_), color.y / (sampling_rate_), color.z / (sampling_rate_));
	return Color4f{ c_srgb(final_color.x),c_srgb(final_color.y),c_srgb(final_color.z),1.0f};
}

Vector3 Shader::get_background(RTCRay dir)
{
	Color3f bckg = spherical_map_->texel(dir.dir_x, dir.dir_y, dir.dir_z);
	return Vector3(c_linear(bckg.r), c_linear(bckg.g), c_linear(bckg.b));
}

Vector3 Shader::traceRay(RTCRay ray, int depth)
{
	RTCRayHit ray_hit = setup_and_intersect_hit(ray);

	if (ray_hit.hit.geomID == RTC_INVALID_GEOMETRY_ID || depth > max_depth_)
	{
		return get_background(ray_hit.ray);
	}
	if (ray_hit.hit.geomID != RTC_INVALID_GEOMETRY_ID)
	{
		RTCGeometry geometry = rtcGetGeometry(scene_, ray_hit.hit.geomID);
		Material* current_material = (Material*)(rtcGetGeometryUserData(geometry));
		
		Vector3 normal;
		rtcInterpolate0(geometry, ray_hit.hit.primID, ray_hit.hit.u, ray_hit.hit.v,
			RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, &normal.x, 3);
		normal.Normalize();

		Vector3 texture_coord;
		rtcInterpolate0(geometry, ray_hit.hit.primID, ray_hit.hit.u, ray_hit.hit.v,
			RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, &texture_coord.x, 2);
		
		Vector3 ray_hit_point = Vector3(ray_hit.ray.org_x + ray_hit.ray.dir_x * ray_hit.ray.tfar, ray_hit.ray.org_y + ray_hit.ray.dir_y * ray_hit.ray.tfar, ray_hit.ray.org_z + ray_hit.ray.dir_z * ray_hit.ray.tfar);
		
		Vector3 light_dir = light_->get_positon() - ray_hit_point;
		light_dir.Normalize();
		
		Vector3 dir = Vector3(ray.dir_x, ray.dir_y, ray.dir_z);
		dir.Normalize();

		Vector3 view_dir = dir * -1.0f;
		view_dir.Normalize();

		if (normal.DotProduct(view_dir) < 0.0f)
		{
			normal *= -1;
		}

		Vector3 light_reflect = generateReflectedVector(light_dir, normal);
			
		/*	= 2.0f * clamp(normal.DotProduct(light_dir),0.0f,1.0f) * normal - light_dir;
		light_reflect.Normalize();*/

		Vector3 half_way = light_dir + view_dir;
		half_way.Normalize();

		float last_ior = ray_hit.ray.time;
		float material_hit_ior = current_material->ior;
		
		//Normal
		if (shading_type_ == 1)
		{
			Vector3 ones = Vector3(1, 1, 1);
			Vector3 return_color = (normal + ones) / 2;
			return Vector3(c_linear(return_color.x), c_linear(return_color.y), c_linear(return_color.z));
		}

		//Lambert
		if (shading_type_ == 2)
		{
			Vector3 ambient = current_material->ambient;

			RTCRayHit ray_hit_shadow = setup_and_intersect_hit(generate_shadow_ray(ray_hit_point));

			float shadow_value;
			if (ray_hit_shadow.hit.geomID == RTC_INVALID_GEOMETRY_ID)
			{
				shadow_value = 1.0f;
			}
			else
			{
				Vector3 to_light = Vector3((light_->get_positon().x - ray_hit_point.x), (light_->get_positon().y - ray_hit_point.y), (light_->get_positon().y - ray_hit_point.y));
				Vector3 to_shadow_hit = Vector3(ray_hit_point.x * ray_hit_shadow.ray.tfar, ray_hit_point.y * ray_hit_shadow.ray.tfar, ray_hit_point.z * ray_hit_shadow.ray.tfar);
				float to_light_size = sqrt(sqr(to_light.x) + sqr(to_light.y) + sqr(to_light.z));
				float to_shadow_hit_size = sqrt(sqr(to_shadow_hit.x) + sqr(to_shadow_hit.y) + sqr(to_shadow_hit.z));
				shadow_value = to_shadow_hit_size / to_light_size;
			}

			Vector3 diffuse;

			Texture* texture = current_material->get_texture(Material::kDiffuseMapSlot);
			if (texture != nullptr)
			{
				Color3f texel = texture->get_texel(texture_coord.x, 1.0f - texture_coord.y);
				diffuse = Vector3(c_linear(texel.r), c_linear(texel.g), c_linear(texel.b)) * light_dir.DotProduct(normal);
			}
			else
			{
				diffuse = current_material->diffuse * light_dir.DotProduct(normal);
				diffuse = Vector3(c_linear(diffuse.x), c_linear(diffuse.y), c_linear(diffuse.z));
			}
			return diffuse;
		}

		//Phong
		if (shading_type_ == 3)
		{
			Vector3 ambient = current_material->ambient;
			ambient = Vector3(c_linear(ambient.x), c_linear(ambient.y), c_linear(ambient.z));
			
			RTCRayHit ray_hit_shadow = setup_and_intersect_hit(generate_shadow_ray(ray_hit_point));

			float shadow_value;
			if (ray_hit_shadow.hit.geomID == RTC_INVALID_GEOMETRY_ID)
			{
				shadow_value = 1.0f;
			}
			else
			{
				Vector3 to_light = Vector3((light_->get_positon().x - ray_hit_point.x), (light_->get_positon().y - ray_hit_point.y), (light_->get_positon().y - ray_hit_point.y));
				Vector3 to_shadow_hit = Vector3(ray_hit_point.x * ray_hit_shadow.ray.tfar, ray_hit_point.y * ray_hit_shadow.ray.tfar, ray_hit_point.z * ray_hit_shadow.ray.tfar);
				float to_light_size = sqrt(sqr(to_light.x) + sqr(to_light.y) + sqr(to_light.z));
				float to_shadow_hit_size = sqrt(sqr(to_shadow_hit.x) + sqr(to_shadow_hit.y) + sqr(to_shadow_hit.z));
				shadow_value = to_shadow_hit_size / to_light_size;
			}

			Vector3 diffuse;

			Texture* texture = current_material->get_texture(Material::kDiffuseMapSlot);
			if (texture != nullptr)
			{
				Color3f texel = texture->get_texel(texture_coord.x, 1.0f - texture_coord.y);
				diffuse = Vector3(c_linear(texel.r), c_linear(texel.g), c_linear(texel.b)) * light_dir.DotProduct(normal);
			}
			else
			{
				diffuse = current_material->diffuse * light_dir.DotProduct(normal);
				diffuse = Vector3(c_linear(diffuse.x), c_linear(diffuse.y), c_linear(diffuse.z));
			}

			Vector3 specular = current_material->specular;
			specular = Vector3(c_linear(specular.x), c_linear(specular.y), c_linear(specular.z));

			specular *= pow(boost::algorithm::clamp(view_dir.DotProduct(light_reflect), 0.0f, 1.0f), current_material->shininess);

			return (ambient + shadow_value * (diffuse + specular));
		}

		//Phong - reflective
		if (current_material->shader_number == 3)
		{
			Vector3 ambient = current_material->ambient;
			ambient = Vector3(c_linear(ambient.x), c_linear(ambient.y), c_linear(ambient.z));

			RTCRayHit ray_hit_shadow = setup_and_intersect_hit(generate_shadow_ray(ray_hit_point));

			float shadow_value;
			if (ray_hit_shadow.hit.geomID == RTC_INVALID_GEOMETRY_ID)
			{
				shadow_value = 1.0f;
			}
			else
			{
				Vector3 to_light = Vector3((light_->get_positon().x - ray_hit_point.x), (light_->get_positon().y - ray_hit_point.y), (light_->get_positon().y - ray_hit_point.y));
				Vector3 to_shadow_hit = Vector3(ray_hit_point.x * ray_hit_shadow.ray.tfar, ray_hit_point.y * ray_hit_shadow.ray.tfar, ray_hit_point.z * ray_hit_shadow.ray.tfar);
				float to_light_size = sqrt(sqr(to_light.x) + sqr(to_light.y) + sqr(to_light.z));
				float to_shadow_hit_size = sqrt(sqr(to_shadow_hit.x) + sqr(to_shadow_hit.y) + sqr(to_shadow_hit.z));
				shadow_value = to_shadow_hit_size / to_light_size;
			}

			Vector3 diffuse;

			Texture* texture = current_material->get_texture(Material::kDiffuseMapSlot);
			if (texture != nullptr)
			{
				Color3f texel = texture->get_texel(texture_coord.x, 1.0f - texture_coord.y);
				diffuse = Vector3(c_linear(texel.r), c_linear(texel.g), c_linear(texel.b)) * light_dir.DotProduct(normal);
			}
			else
			{
				diffuse = current_material->diffuse * light_dir.DotProduct(normal);
				diffuse = Vector3(c_linear(diffuse.x), c_linear(diffuse.y), c_linear(diffuse.z));
			}

			Vector3 specular = current_material->specular;
			specular = Vector3(c_linear(specular.x), c_linear(specular.y), c_linear(specular.z));

			specular *= pow(boost::algorithm::clamp(view_dir.DotProduct(light_reflect), 0.0f, 1.0f), current_material->shininess);

			float n1 = ray_hit.ray.time;
			float n2 = current_material->ior;

			float R_0 = pow(((n1 - n2) / (n1 + n2)), 2);
			float R = R_0 + (1 - R_0) * pow((1 - clamp(normal.DotProduct(light_dir),0.0f,1.0f)), 5);

			Vector3 reflect = generateReflectedVector(view_dir, normal);

			return (ambient + shadow_value * (diffuse + specular)) + (traceRay(generateSecondaryRay(ray_hit_point,reflect,n2), depth + 1) * R);




			//Vector3 ambient = current_material->ambient;

			//RTCRayHit ray_hit_shadow = setup_and_intersect_hit(generate_shadow_ray(ray_hit_point));

			//float shadow_value;
			//if (ray_hit_shadow.hit.geomID == RTC_INVALID_GEOMETRY_ID)
			//{
			//	shadow_value = 1.0f;
			//}
			//else
			//{
			//	shadow_value = 0.25f;
			//}

			//Vector3 diffuse;

			//Texture *texture = current_material->get_texture(Material::kDiffuseMapSlot);
			//if (texture != nullptr)
			//{
			//	Color3f texel = texture->get_texel(texture_coord.x, 1.0f - texture_coord.y);
			//	diffuse =  Vector3(c_linear(texel.r), c_linear(texel.g), c_linear(texel.b)) * light_dir.DotProduct(normal);
			//}
			//else
			//{
			//	diffuse = current_material->diffuse * light_dir.DotProduct(normal);
			//	diffuse = Vector3(c_linear(diffuse.x), c_linear(diffuse.y), c_linear(diffuse.z));
			//}

			//Vector3 specular = current_material->specular * pow(max(0.0f, boost::algorithm::clamp(light_reflect.DotProduct(view_dir),0.0f,1.0f)), current_material->shininess);

			////Schlick's approximation
			//float n1 = ray_hit.ray.time;
			//float n2 = current_material->ior;

			//float R_0 = pow(((n1 - n2) / (n1 + n2)), 2);
			//float R = R_0 + (1 - R_0) * pow((1 - boost::algorithm::clamp(normal.DotProduct(view_dir), 0.0f, 1.0f)), 5);

			//return (ambient + shadow_value * (diffuse + specular)) + (traceRay(generate_reflection(view_dir,normal, ray_hit_point, ray_hit.ray.time),depth+1) * R);
		}

		//Glass
		if (current_material->shader_number == 4)
		{
			//float n1 = last_ior;
			//float n2 = material_hit_ior;

			//if (n1 != IOR_AIR)
			//{
			//	n2 = IOR_AIR;
			//}
			//float n1_over_n2 = n1 / n2;
			//float Q1 = normal.DotProduct(view_dir);

			//Vector3 omega_i;
			//omega_i = generateReflectedVector(view_dir, normal);

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
			reflection = generateReflectedVector(view_dir, normal);

			//total reflection
			if (under_sqrt < 0.0f)
			{
				return traceRay(generateSecondaryRay(ray_hit_point, reflection, n1), depth + 1);
			}

			Vector3 refract_dir;

			refract_dir = (n1_over_n2 * dir) - (n1_over_n2 * dir.DotProduct(normal) + sqrt(under_sqrt))*normal;
			refract_dir.Normalize();

			float cos_theta1 = view_dir.DotProduct(normal);
			float cos_theta2 = sqrt(1 - sqr(n1_over_n2) * (1 - sqr(cos_theta1)));

			float Rs = sqr((n2 * cos_theta2 - n1 * cos_theta1) / (n2 * cos_theta2 + n1 * cos_theta1));
			float Rp = sqr((n2 * cos_theta1 - n1 * cos_theta2) / (n2 * cos_theta1 + n1 * cos_theta2));

			float R = 0.5f * (Rs + Rp);

			Vector3 diffuse = current_material->diffuse;
			diffuse = Vector3(c_linear(diffuse.x), c_linear(diffuse.y), c_linear(diffuse.z));
			Vector3 reflected_part = traceRay(generateSecondaryRay(ray_hit_point, reflection, n1), depth + 1) * R;
			Vector3 refracted_part = traceRay(generateSecondaryRay(ray_hit_point, refract_dir, n2), depth + 1) * (1.0f - R) * diffuse;
			Vector3 L_r = reflected_part + refracted_part;
			return L_r;

















			/*float n1 = ray_hit.ray.time;
			float n2 = current_material->ior;

			float R_0 = pow(((n1 - n2) / (n1 + n2)), 2);
			float R = R_0 + (1 - R_0) * pow((1 - boost::algorithm::clamp(normal.DotProduct(view_dir), 0.0f, 1.0f)), 5);

			Vector3 diffuse = current_material->diffuse;
			diffuse = Vector3(c_linear(diffuse.x), c_linear(diffuse.y), c_linear(diffuse.z));

			return (traceRay(generate_reflection(view_dir,normal,ray_hit_point, ray_hit.ray.time),depth+1) * R) + (diffuse * traceRay(generate_refraction(dir, normal, ray_hit.ray.time, current_material->ior, ray_hit_point), depth + 1) * (1.0f - R));*/
		}
	}
}

RTCRayHit Shader::setup_and_intersect_hit(RTCRay ray)
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

RTCRay Shader::generate_shadow_ray(Vector3 ray_hit_point)
{
	RTCRay shadow_ray = RTCRay();

	shadow_ray.org_x = ray_hit_point.x; // ray origin
	shadow_ray.org_y = ray_hit_point.y;
	shadow_ray.org_z = ray_hit_point.z;
	shadow_ray.tnear = 0.01f; // start of ray segment

	shadow_ray.dir_x = light_->get_positon().x - ray_hit_point.x; // ray direction
	shadow_ray.dir_y = light_->get_positon().y - ray_hit_point.y;
	shadow_ray.dir_z = light_->get_positon().z - ray_hit_point.z;
	shadow_ray.time = 0.0f; // time of this ray for motion blur

	shadow_ray.tfar = 1.0f; // end of ray segment (set to hit distance)

	shadow_ray.mask = 0; // can be used to mask out some geometries for some rays
	shadow_ray.id = 0; // identify a ray inside a callback function
	shadow_ray.flags = 0; // reserved

	return shadow_ray;
}

RTCRay Shader::generate_reflection(Vector3 view_dir, Vector3 normal, Vector3 ray_hit_point,float ior)
{
	Vector3 dir = 2.0f * (view_dir * normal)* normal - view_dir;
	dir.Normalize();
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

RTCRay Shader::generate_refraction(Vector3 dir, Vector3 normal, float last_ior, float material_hit_ior, Vector3 ray_hit_point)
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

RTCRay Shader::generateSecondaryRay(Vector3 position, Vector3 direction, float ior)
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

Vector3 Shader::generateReflectedVector(Vector3 vec, Vector3 normal)
{
	Vector3 reflect = 2.0f * (clamp(vec.DotProduct(normal), 0.0f, 1.0f)) * normal - vec;
	reflect.Normalize();
	return reflect;
}


float Shader::c_srgb(float c_linear, float gamma)
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

float Shader::c_linear(float c_srgb, float gamma)
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
