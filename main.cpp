#include <vector>
#include <iostream>

#include "model.h"
#include "our_gl.h"

#define CLAMP(t) ((t>1.f)?1.f:((t<0.f)?0.f:t))

Model *model = NULL;
const int width  = 600;
const int height = 600;

//投影，坐标变换,光线相关变量
Vec3f light_dir(1,  1,  1);
Vec3f       eye(0, -1,  3);
Vec3f    center(0,  0,  0);
Vec3f        up(0,  1,  0);

struct Shader : public IShader//目前方案为GouraudShading
{
    Vec3f varying_intensity; // write by vertex shader, read by fragment shader
    mat<2, 3, float>varying_uv;
    mat<4, 4, float> uniform_M; //Projection*ModelView
    mat<4, 4, float> uniform_MIT; // (Projection*ModelView).invert_transpose()

    //iface三角面片索引，nthvert对应f数据中的索引号
    virtual Vec4f vertex(int iface, int nthvert) {//顶点着色器
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));
        Vec4f gl_vertex = embed<4>(model->vert(iface,nthvert));
        return ViewPort * Projection * ModelView * gl_vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor& color) {//片段着色器
        Vec2f uv = varying_uv * bar;//纹理坐标插值
        Vec3f n = proj<3>(uniform_MIT * embed<4>(model->normal(uv))).normalize(); // transform normal vector
        Vec3f l = proj<3>(uniform_M * embed<4>(light_dir)).normalize(); // transfrom light direction
        Vec3f r = (n * (n * l * 2.f) - l).normalize(); // reflected light
        float spec = pow(std::max(r.z, 0.0f), model->specular(uv)); //镜面反射 we're looking from z-axis
        float diff = std::max(0.f, n * l);//漫反射
        TGAColor c = model->diffuse(uv);//取纹理
        color = c;
        for (int i = 0; i < 3; i++) color[i] = std::min<float>(5 + c[i] * (diff + .6 * spec), 255);
        return false;
    }
};

int main(int argc, char** argv) {
    //绘制模型
    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("obj/african_head.obj");
    }
    
    lookat(eye, center, up);
    viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
    projection(-1.f / (eye - center).norm());
    light_dir.normalize();

    TGAImage image(width, height, TGAImage::RGB);
    TGAImage zbuffer(width, height, TGAImage::GRAYSCALE);

    Shader shader;
    shader.uniform_M = Projection * ModelView;
    shader.uniform_MIT = (Projection * ModelView).invert_transpose();
    for (int i = 0; i < model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec4f screen_coords[3];
        for (int j = 0; j < 3; j++) {
            screen_coords[j] = shader.vertex(i, j);//处理每个三角形，将三个顶点的intensity数据算好，返回转换后坐标
        }
        triangle(screen_coords, shader, image, zbuffer);
    }
    
    image.flip_vertically(); // 上下翻转
    zbuffer.flip_vertically();
    image.write_tga_file("output_Phonglight.tga");
    zbuffer.write_tga_file("zbuffer_Phonglight.tga");

    delete model;
    return 0;
}

