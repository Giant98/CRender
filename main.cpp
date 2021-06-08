#include <vector>
#include <iostream>

#include "model.h"
#include "our_gl.h"

#define CLAMP(t) ((t>1.f)?1.f:((t<0.f)?0.f:t))

Model *model = NULL;
const int width  = 600;
const int height = 600;

//ͶӰ������任,������ر���
Vec3f light_dir(1,  1,  1);
Vec3f       eye(0, -1,  3);
Vec3f    center(0,  0,  0);
Vec3f        up(0,  1,  0);

struct FlatShader : public IShader
{
    mat<3, 3, float>varying_tri;//���ǽ����Ϊ varying_tri ����Ϊ varying��GLSL�еı�����

    virtual Vec4f vertex(int iface, int nthvert) {
        Vec4f gl_vertex = embed<4>(model->vert(iface,nthvert));
        gl_vertex = Projection * ModelView * gl_vertex; 
        varying_tri.set_col(nthvert, proj<3>(gl_vertex / gl_vertex[3]));//��¼transform��������Σ����������Ƭ����ɫ�����������ε���ɫ��ʱ��ʹ����ʹ��
        gl_vertex = ViewPort * gl_vertex;
        return gl_vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor& color) {
        Vec3f n = cross(varying_tri.col(1) - varying_tri.col(0), varying_tri.col(2) -
            varying_tri.col(0)).normalize();//���㷨����
        float intensity = CLAMP(n * light_dir);// ��ǿ��clamp��0��1֮��
        color = TGAColor(255, 255, 255) * intensity;
        return false;
    }
};

int main(int argc, char** argv) {
    //����ģ��
    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("obj/african_head.obj");
    }
    
    lookat(eye, center, up);
    viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
    projection(-1.f / (eye - center).norm());

    TGAImage image(width, height, TGAImage::RGB);
    TGAImage zbuffer(width, height, TGAImage::GRAYSCALE);

    FlatShader shader;
    for (int i = 0; i < model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec4f screen_coords[3];
        for (int j = 0; j < 3; j++) {
            screen_coords[j] = shader.vertex(i, j);
        }
        triangle(screen_coords, shader, image, zbuffer);
    }
    
    image.flip_vertically(); // ���·�ת
    zbuffer.flip_vertically();
    image.write_tga_file("output_advance.tga");
    zbuffer.write_tga_file("zbuffer_advance.tga");

    delete model;
    return 0;
}

