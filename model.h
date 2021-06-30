#ifndef __MODEL_H__
#define __MODEL_H__

#include <vector>
#include <string>
#include "geometry.h"
#include "tgaimage.h"

class Model {
private:
	std::vector<Vec3f> verts_;//顶点数据
	std::vector<Vec2f>uvs_;//纹理坐标
	std::vector<Vec3f>norms_;//顶点法向量
	std::vector<std::vector<Vec3i>> faces_;//面数据
	TGAImage diffusemap_;
	TGAImage normalmap_;
	TGAImage specularmap_;
	void load_texture(std::string filename, const char* suffix, TGAImage& img);
public:
	Model(const char* filename);
	~Model();
	int nverts();
	int nfaces();
	Vec2f uv(int iface, int nthvert);
	Vec3f vert(int iface, int nthvert);
	Vec3f normal(int iface, int nthvert);
	Vec3f normal(Vec2f uv);
	std::vector<int> face(int idx);
	TGAColor diffuse(Vec2f uv);
	float specular(Vec2f uv);
};

#endif //__MODEL_H__