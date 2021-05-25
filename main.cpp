#include <cstdlib>
#include <limits>
#include <iostream>
#include "model.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
const TGAColor green = TGAColor(0, 255, 0, 255);
Model *model = NULL;
const int width  = 800;
const int height = 800;

//ͶӰ������任,������ر���
Vec3f light_dir(0, 0, -1);
Vec3f       eye(0, 0,  3);
Vec3f    center(0, 0,  0);
Vec3f        up(0, 1,  0);

Matrix ModelView;//ģ��(�ֲ��ռ䵽����ռ�����)
Matrix ViewPort;//�۲�(�任��camera����,Ҳ���ǹ۲�ռ�)
Matrix Projection;//ͶӰ����(��3D����ͶӰ������ӳ�䵽2D�ı�׼���豸����ϵ��,�ü�������Χ������,�������͸��ͶӰ)

//���ߺ���
void line(Vec2i p0, Vec2i p1, TGAImage& image, TGAColor color) {
    bool steep = false;
    if (std::abs(p0.x - p1.x) < std::abs(p0.y - p1.y)) {//��ֹб�ʴ���1
        std::swap(p0.x, p0.y);
        std::swap(p1.x, p1.y);
        steep = true;
    }
    if (p0.x > p1.x) {//��ֹб���Ǹ�
        std::swap(p0, p1);
    }

    for (int x = p0.x; x <= p1.x; x++) {
        float t = (x - p0.x) / (float)(p1.x - p0.x);
        int y = p0.y * (1. - t) + p1.y * t + .5;
        if (steep) {
            image.set(y, x, color);
        }
        else {
            image.set(x, y, color);
        }
    }
}

//����OpenGL LookAt,(���λ��pos��Ŀ��λ��target,���������up)
void lookat(Vec3f eye, Vec3f center, Vec3f up) {
    Vec3f z = (eye - center).normalize();
    Vec3f x = cross(up, z).normalize();
    Vec3f y = cross(z, x).normalize();
    Matrix Minv = Matrix::identity();//��λ����
    Matrix Tr = Matrix::identity();
    for (int i = 0; i < 3; i++) {
        Minv[0][i] = x[i];//��ת����
        Minv[1][i] = y[i];
        Minv[2][i] = z[i];
        Tr[i][3] = -center[i];//ƽ�ƾ���
    }
    ModelView = Minv * Tr;
}
//������ģ�Ͷ���[-1,1]*[-1,1]*[-1,1]�������У����������ӳ�䵽λ��[x,x+w]*[y,y+h]*[0,d]��
void viewport(int x, int y, int w, int h) {
    ViewPort = Matrix::identity();
    ViewPort[0][3] = x + w / 2.f;
    ViewPort[1][3] = y + h / 2.f;
    ViewPort[2][3] = 1.f;
    ViewPort[0][0] = w / 2.f;
    ViewPort[1][1] = h / 2.f;
    ViewPort[2][2] = 1.f;
}
//֤�����̳�https://github.com/ssloy/tinyrenderer/wiki/Lesson-4:-Perspective-projection
//coeffΪcamera����(0��0��c)
void projection(float coeff) {
    Projection = Matrix::identity();
    Projection[3][2] = coeff;
}

Vec3f barycentric(Vec3f A, Vec3f B, Vec3f C, Vec3f P) {
    Vec3f s[2];
    for (int i = 2; i--; ) {
        s[i][0] = C[i] - A[i];
        s[i][1] = B[i] - A[i];
        s[i][2] = A[i] - P[i];
    }
    Vec3f u = cross(s[0], s[1]);
    if (std::abs(u[2]) > 1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
        return Vec3f(1.f - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z);
    return Vec3f(-1, 1, 1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}

void triangle(Vec3f* pts, Vec2f* texts, float* zbuffer, TGAImage& image, float intensity) {
    Vec2f bboxmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    Vec2f clamp(image.get_width() - 1, image.get_height() - 1);
    //���������ι������η�Χ
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            bboxmin[j] = std::max(0.f, std::min(bboxmin[j], pts[i][j]));
            bboxmax[j] = std::min(clamp[j], std::max(bboxmax[j], pts[i][j]));
        }
    }
    Vec3f P;
    for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
        for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
            Vec3f bc_screen = barycentric(pts[0], pts[1], pts[2], P);//�ҵ���������ģ�1-u-v, u, v��
            if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0) continue;//�ж��Ƿ����������ڲ����ڲ�����ɫ
            P.z = 0;
            Vec2f Ptext(0, 0);
            for (int i = 0; i < 3; i++) P.z += pts[i][2] * bc_screen[i];//����õ�P�����ֵ
            for (int i = 0; i < 3; i++) Ptext[0] += texts[i][0] * bc_screen[i];
            for (int i = 0; i < 3; i++) Ptext[1] += texts[i][1] * bc_screen[i];
            if (zbuffer[int(P.x + P.y * width)] < P.z) {
                TGAColor color = model->diffuse(Ptext);
                image.set(P.x, P.y, color);
                zbuffer[int(P.x + P.y * width)] = P.z;
            }
        }
    }
}

Vec3f world2screen(Vec3f v) {
    Vec4f gl_vertex = embed<4>(v); // embed Vec3f to homogenius coordinates
    gl_vertex = ViewPort * Projection * ModelView * gl_vertex; // transform it to screen coordinates
    Vec3f v3 = proj<3>(gl_vertex / gl_vertex[3]); // transfromed vec3f vertex
    return Vec3f(int(v3.x + .5), int(v3.y + .5), v3.z);
}

int main(int argc, char** argv) {
    TGAImage image(width, height, TGAImage::RGB);
    //����ģ��
    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("obj/african_head.obj");
    }
    
    float* zbuffer = new float[width * height];
    for (int i = width * height; i--; zbuffer[i] = -std::numeric_limits<float>::max());
    //�任����
    lookat(eye, center, up);
    viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
    projection(-1.f / 3);

    //������
    for (int i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec3f screen_coords[3];
        Vec3f world_coords[3];
        Vec2f texts[3];
        for (int j=0; j<3; j++) {
            world_coords[j] = model->vert(face[2 * j]);
            screen_coords[j] = world2screen(world_coords[j]);//ת��Ϊ��Ļ�ռ�����
            texts[j] = model->uv(face[2 * j + 1]);
        }
        Vec3f n = cross((world_coords[2] - world_coords[0]), (world_coords[1] - world_coords[0]));
        n.normalize();
        float intensity = n * light_dir;
        if (intensity > 0) {
            triangle(screen_coords, texts, zbuffer, image, intensity);
        }
    }
    
    image.flip_vertically(); // ���·�ת
    image.write_tga_file("output_Perspective.tga");
    //delete model;
    return 0;
}

