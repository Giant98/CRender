#include <iostream>
#include <fstream>
#include <sstream>
#include "model.h"

Model::Model(const char* filename) : verts_(), faces_(), uvs_(), diffusemap_() {
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
        else if (!line.compare(0, 2, "f ")) {
            std::vector<int> f;
            int itrash, idx, idxt;
            iss >> trash;
            while (iss >> idx >> trash >> idxt >> trash >> itrash) {
                idx--; // in wavefront obj all indices start at 1, not zero
                idxt--; // in wavefront obj all indices start at 1, not zero
                f.push_back(idx);
                f.push_back(idxt);
            }
            faces_.push_back(f);
        }    
    }
    load_texture(filename, "_diffuse.tga", diffusemap_);
    std::cerr << "# v# " << verts_.size() << " f# " << faces_.size() << std::endl;
}

Model::~Model() {
}

int Model::nverts() {
    return (int)verts_.size();
}

Vec3f Model::vert(int i) {
    return verts_[i];
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

TGAColor Model::diffuse(Vec2f uv) {
    Vec2i uvwh(uv[0] * diffusemap_.get_width(), uv[1] * diffusemap_.get_height());
    return diffusemap_.get(uvwh[0], uvwh[1]);
}

int Model::nfaces() {
    return (int)faces_.size();
}

std::vector<int> Model::face(int idx) {
    return faces_[idx];
}

Vec2f Model::uv(int i) {
    return uvs_[i];
}