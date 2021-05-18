#include <vector>
#include <cmath>
#include <limits>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

const int width = 800;
const int height = 800;
const int depth = 255;

Model* model = NULL;
int* zbuffer = NULL;
Vec3f light_dir(0, 0, -1);

void triangle(Vec3i t0, Vec3i t1, Vec3i t2, Vec2i uv0, Vec2i uv1, Vec2i uv2, TGAImage& image, float intensity, int* zbuffer) {
    if (t0.y == t1.y && t0.y == t2.y) return; // i dont care about degenerate triangles
    if (t0.y > t1.y) { std::swap(t0, t1); std::swap(uv0, uv1); }
    if (t0.y > t2.y) { std::swap(t0, t2); std::swap(uv0, uv2); }
    if (t1.y > t2.y) { std::swap(t1, t2); std::swap(uv1, uv2); }

void triangle(Vec3f* pts, Vec2f* texts, float* zbuffer, TGAImage& image, float intensity) {
    Vec2f bboxmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    Vec2f clamp(image.get_width() - 1, image.get_height() - 1);
    //根据三角形构建矩形范围
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            bboxmin[j] = std::max(0.f, std::min(bboxmin[j], pts[i][j]));
            bboxmax[j] = std::min(clamp[j], std::max(bboxmax[j], pts[i][j]));
        }
    }
    Vec3f P;
    for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
        for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
            Vec3f bc_screen = barycentric(pts[0], pts[1], pts[2], P);//找到重心坐标的（1-u-v, u, v）
            if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0) continue;//判断是否在三角形内部，内部才着色
            P.z = 0;
            Vec2f Ptext(0, 0);
            for (int i = 0; i < 3; i++) P.z += pts[i][2] * bc_screen[i];//计算得到P点深度值
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

int main(int argc, char** argv) {
    if (2 == argc) {
        model = new Model(argv[1]);
    }
    else {
        model = new Model("obj/african_head.obj");
    }

    zbuffer = new int[width * height];
    for (int i = 0; i < width * height; i++) {
        zbuffer[i] = std::numeric_limits<int>::min();
    }

    //漫反射
    Vec3f light_dir(0, 0, -1);
    for (int i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec3f screen_coords[3];
        Vec3f world_coords[3];
        Vec2f texts[3];
        for (int j=0; j<3; j++) {
            world_coords[j] = model->vert(face[2 * j]);
            screen_coords[j] = world2screen(world_coords[j]);//转换为屏幕空间坐标
            texts[j] = model->uv(face[2 * j + 1]);
        }
        Vec3f n = cross((world_coords[2] - world_coords[0]), (world_coords[1] - world_coords[0]));
        n.normalize();
        float intensity = n * light_dir;
        if (intensity > 0) {
            triangle(screen_coords, texts, zbuffer, image, intensity);
        }
    }
    
    image.flip_vertically(); // 上下翻转
    image.write_tga_file("output.tga");
    //delete model;
    return 0;
}