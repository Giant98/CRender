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

//投影，坐标变换,光线相关变量
Vec3f light_dir(0, 0, -1);
Vec3f       eye(0, 0,  3);
Vec3f    center(0, 0,  0);
Vec3f        up(0, 1,  0);

Matrix ModelView;//模型(局部空间到世界空间坐标)
Matrix ViewPort;//观察(变换到camera坐标,也就是观察空间)
Matrix Projection;//投影矩阵(将3D坐标投影到容易映射到2D的标准化设备坐标系中,裁剪超出范围的坐标,有正射和透视投影)

//画线函数
void line(Vec2i p0, Vec2i p1, TGAImage& image, TGAColor color) {
    bool steep = false;
    if (std::abs(p0.x - p1.x) < std::abs(p0.y - p1.y)) {//防止斜率大于1
        std::swap(p0.x, p0.y);
        std::swap(p1.x, p1.y);
        steep = true;
    }
    if (p0.x > p1.x) {//防止斜率是负
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

//类似OpenGL LookAt,(相机位置pos，目标位置target,相机上向量up)
void lookat(Vec3f eye, Vec3f center, Vec3f up) {
    Vec3f z = (eye - center).normalize();
    Vec3f x = cross(up, z).normalize();
    Vec3f y = cross(z, x).normalize();
    Matrix Minv = Matrix::identity();//单位矩阵
    Matrix Tr = Matrix::identity();
    for (int i = 0; i < 3; i++) {
        Minv[0][i] = x[i];//旋转矩阵
        Minv[1][i] = y[i];
        Minv[2][i] = z[i];
        Tr[i][3] = -center[i];//平移矩阵
    }
    ModelView = Minv * Tr;
}
//现在有模型都在[-1,1]*[-1,1]*[-1,1]正方体中，我们想把它映射到位置[x,x+w]*[y,y+h]*[0,d]中
void viewport(int x, int y, int w, int h) {
    ViewPort = Matrix::identity();
    ViewPort[0][3] = x + w / 2.f;
    ViewPort[1][3] = y + h / 2.f;
    ViewPort[2][3] = 255 / 2.f;//方便更好地运用齐次坐标，并且可以生成灰度图像
    ViewPort[0][0] = w / 2.f;
    ViewPort[1][1] = h / 2.f;
    ViewPort[2][2] = 255 / 2.f;
}
//证明见教程https://github.com/ssloy/tinyrenderer/wiki/Lesson-4:-Perspective-projection
//coeff为camera处在(0，0，c)
void projection(float coeff) {
    Projection = Matrix::identity();
    Projection[3][2] = coeff;
}

Vec3f barycentric(Vec2f A, Vec2f B, Vec2f C, Vec2f P) {
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

void triangle(Vec4f* pts, TGAImage& image, TGAImage& zbuffer, TGAColor color) {
    Vec2f bboxmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    //根据三角形构建矩形范围
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            bboxmin[j] = std::min(bboxmin[j], pts[i][j] / pts[i][3]);
            bboxmax[j] = std::max(bboxmax[j], pts[i][j] / pts[i][3]);
        }
    }
    Vec2i P;
    for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
        for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
            Vec3f c = barycentric(proj<2>(pts[0] / pts[0][3]), proj<2>(pts[1] / pts[1][3]),
                proj<2>(pts[2] / pts[2][3]), P);//找到重心坐标的（1-u-v, u, v）
            float z = pts[0][2] * c.x + pts[1][2] * c.y + pts[2][2] * c.z;
            float w = pts[0][3] * c.x + pts[1][3] * c.y + pts[2][3] * c.z;
            int frag_depth = std::max(0, std::min(255, int(z / w + .5)));
            if (c.x < 0 || c.y < 0 || c.z < 0 || zbuffer.get(P.x, P.y)[0] > frag_depth) continue;//判断是否在三角形内部，内部才着色//计算得到P点深度值
            //for (int i = 0; i < 3; i++) Ptext[0] += texts[i][0] * bc_screen[i];
            //for (int i = 0; i < 3; i++) Ptext[1] += texts[i][1] * bc_screen[i];
            image.set(P.x, P.y, color);
            zbuffer.set(P.x, P.y, TGAColor(frag_depth));
        }
    }
}

Vec4f world2screen(Vec3f v) {
    Vec4f gl_vertex = embed<4>(v); // embed Vec3f to homogenius coordinates
    gl_vertex = ViewPort * Projection * ModelView * gl_vertex; // transform it to screen coordinates
    return gl_vertex;
}

int main(int argc, char** argv) {
    TGAImage image(width, height, TGAImage::RGB);
    TGAImage zbuffer(width, height, TGAImage::GRAYSCALE);
    //绘制模型
    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("obj/african_head.obj");
    }
    
    lookat(eye, center, up);
    viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
    projection(-1.f / 3);

    //漫反射
    for (int i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec4f screen_coords[3];
        Vec3f world_coords[3];
        //Vec2f texts[3];
        for (int j=0; j<3; j++) {
            world_coords[j] = model->vert(face[2 * j]);
            screen_coords[j] = world2screen(world_coords[j]);//转换为屏幕空间坐标
            //texts[j] = model->uv(face[2 * j + 1]);
        }
        Vec3f n = cross((world_coords[2] - world_coords[0]), (world_coords[1] - world_coords[0]));
        n.normalize();
        float intensity = n * light_dir;
        if (intensity > 0) {
            triangle(screen_coords, image, zbuffer, TGAColor(intensity * 255, intensity * 255, intensity * 255, 255));
        }
    }
    
    image.flip_vertically(); // 上下翻转
    zbuffer.flip_vertically();
    image.write_tga_file("output_simple.tga");
    zbuffer.write_tga_file("zbuffer.tga");
    //delete model;
    return 0;
}

