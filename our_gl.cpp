#include <cmath>
#include <limits>
#include <cstdlib>
#include "our_gl.h"

Matrix ModelView;//模型(局部空间到世界空间坐标)
Matrix ViewPort;//观察(变换到camera坐标,也就是观察空间)
Matrix Projection;//投影矩阵(将3D坐标投影到容易映射到2D的标准化设备坐标系中,裁剪超出范围的坐标,有正射和透视投影)

IShader::~IShader() {}

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

void triangle(Vec4f* pts, IShader& shader, TGAImage& image, TGAImage& zbuffer) {
    Vec2f bboxmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    //根据三角形构建矩形范围
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            // x/w y/w
            bboxmin[j] = std::min(bboxmin[j], pts[i][j] / pts[i][3]);
            bboxmax[j] = std::max(bboxmax[j], pts[i][j] / pts[i][3]);
        }
    }

    Vec2i P;
    TGAColor color;
    for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
        for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
            Vec3f c = barycentric(proj<2>(pts[0] / pts[0][3]), proj<2>(pts[1] / pts[1][3]),
                proj<2>(pts[2] / pts[2][3]), P);//找到重心坐标的（1-u-v, u, v）
            float z = pts[0][2] * c.x + pts[1][2] * c.y + pts[2][2] * c.z;
            float w = pts[0][3] * c.x + pts[1][3] * c.y + pts[2][3] * c.z;
            int frag_depth = std::max(0, std::min(255, int(z / w + .5)));
            if (c.x < 0 || c.y < 0 || c.z < 0 || zbuffer.get(P.x, P.y)[0] > frag_depth) continue;
            //for (int i = 0; i < 3; i++) Ptext[0] += texts[i][0] * bc_screen[i];
            //for (int i = 0; i < 3; i++) Ptext[1] += texts[i][1] * bc_screen[i];
            bool discard = shader.fragment(c, color);
            if (!discard) {
                zbuffer.set(P.x, P.y, TGAColor(frag_depth));
                image.set(P.x, P.y, color);
            }
        }
    }
}
