#include <iostream>
#include <fstream>
#include <sstream>
#include "model.h"

Model::Model(const char* filename) : verts_(), uvs_(), norms_(), faces_(), diffusemap_() {
    std::ifstream in;
    in.open(filename, std::ifstream::in);
    if (in.fail()) return;
    std::string line;
    while (!in.eof()) {
        std::getline(in, line);
        std::istringstream iss(line.c_str());
        char trash;
        if (!line.compare(0, 2, "v ")) {
            iss >> trash;
            Vec3f v;
            for (int i = 0; i < 3; i++) iss >> v[i];
            verts_.push_back(v);
        }
        else if (!line.compare(0, 2, "vt")) {
            iss >> trash >> trash;
            Vec2f uv;
            for (int i = 0; i < 2; i++)iss >> uv[i];
            uvs_.push_back(uv);
        }
        else if (!line.compare(0, 2, "vn")) {
            iss >> trash >> trash;
            Vec3f n;
            for (int i = 0; i < 3; i++) iss >> n[i];
            norms_.push_back(n);
        }
        else if (!line.compare(0, 2, "f ")) {
            std::vector<Vec3i> f;
            Vec3i tmp;
            iss >> trash;
            while (iss >> tmp[0] >> trash >> tmp[1] >> trash >> tmp[2]) {
                for (int i = 0; i < 3; i++) tmp[i]--; // in wavefront obj all indices start at 1, not zero
                f.push_back(tmp);
            }
            faces_.push_back(f);
        }    
    }
    load_texture(filename, "_diffuse.tga", diffusemap_);
    load_texture(filename, "_nm.tga", normalmap_);
    std::cerr << "# v#" << verts_.size() << " f# " << faces_.size() << " vt# " <<
        uvs_.size() << " vn# " << norms_.size() << std::endl;
}

Model::~Model() {
}

int Model::nverts() {
    return (int)verts_.size();
}

int Model::nfaces() {
    return (int)faces_.size();
}

Vec3f Model::vert(int iface, int nthvert) {//获取顶点编号
    return verts_[faces_[iface][nthvert][0]];
}

Vec2f Model::uv(int iface, int nthvert) {//获取顶点对应纹理坐标编号
    return uvs_[faces_[iface][nthvert][1]];
}

Vec3f Model::normal(Vec2f uvf) {
    Vec2i uv(uvf[0] * normalmap_.get_width(), uvf[1] * normalmap_.get_height());
    TGAColor c = normalmap_.get(uv[0], uv[1]);
    Vec3f res;
    // notice TGAColor is bgra, and in byte
    for (int i = 0; i < 3; i++)
        res[2 - i] = (float)c[i] / 255.f * 2.f - 1.f;
    return res;
}

Vec3f Model::normal(int iface, int nthvert) {//获取顶点法向量编号
    int idx = faces_[iface][nthvert][2];
    return norms_[idx].normalize();
}

void Model::load_texture(std::string filename, const char* suffix, TGAImage& img) {
    std::string textfile(filename);
    size_t dot = textfile.find_last_of(".");
    if (dot != std::string::npos) {
        textfile = textfile.substr(0, dot) + std::string(suffix);
        std::cout << "textfile file" << textfile << "loading " <<
            (img.read_tga_file(textfile.c_str()) ? "ok" : "failed") << std::endl;
        img.flip_vertically();
    }
}

std::vector<int> Model::face(int idx) {
    std::vector<int> face;
    for (int i = 0; i < (int)faces_[idx].size(); i++) face.push_back(faces_[idx][i][0]);
    return face;
}

TGAColor Model::diffuse(Vec2f uv) {
    Vec2i uvwh(uv[0] * diffusemap_.get_width(), uv[1] * diffusemap_.get_height());
    return diffusemap_.get(uvwh[0], uvwh[1]);
}