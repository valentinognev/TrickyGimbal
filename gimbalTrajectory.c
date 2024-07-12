#include <math.h>
#include <stdio.h>

#define PI 3.14159265
#define DEG2RAD(deg) ((deg) * PI / 180.0)
#define RAD2DEG(rad) ((rad) * 180.0 / PI)

// Assuming you have implemented these functions
void roty(double angle, double matrix[3][3]);
void rotz(double angle, double matrix[3][3]);
void cross(double a[3], double b[3], double result[3]);
double dot(double a[3], double b[3]);
void axang2rotm(double axis[3], double angle, double matrix[3][3]);
void matvecmul(const double mat[3][3], const double vec[3], double result[3]);

int main() 
{
    double gamma = DEG2RAD(30);
    double elevation1 = DEG2RAD(20);
    double azimuth1 = DEG2RAD(0);
    double elevation2 = DEG2RAD(30);
    double azimuth2 = DEG2RAD(70);
    double rotElev1[3][3], rotAz1[3][3], rotElev2[3][3], rotAz2[3][3];
    double rot1[3][3], rot2[3][3];
    double rinit[3] = {0, 0, 1};
    double r1[3], r2[3];
    double r1r2axis[3], theta;
    int N = 10;
    double dtheta, rotdtheta[3][3];
    double elevation[N], azimuth[N], alpha2[N], eta, alpha1[N];

    roty(elevation1, rotElev1);
    rotz(azimuth1, rotAz1);
    roty(elevation2, rotElev2);
    rotz(azimuth2, rotAz2);

    // Assuming you have implemented matrix multiplication
    matmul(rotAz1, rotElev1, rot1);
    matmul(rotAz2, rotElev2, rot2);

    // Assuming you have implemented matrix-vector multiplication
    matvecmul(rot1, rinit, r1);
    matvecmul(rot2, rinit, r2);

    cross(r1, r2, r1r2axis);
    double norm = sqrt(dot(r1r2axis, r1r2axis));
    for (int i = 0; i < 3; i++) {
        r1r2axis[i] /= norm;
    }

    theta = acos(dot(r1, r2));
    dtheta = theta / N;

    axang2rotm(r1r2axis, dtheta, rotdtheta);

    double curr[3] = {r1[0], r1[1], r1[2]};
    for (int i = 0; i < N; i++) 
    {
        double ncurr[3]={curr[0],curr[1],curr[2]};

        matvecmul(rotdtheta, ncurr, curr);
        elevation[i] = acos(dot(curr, (double[]){0, 0, 1}));
        azimuth[i] = atan2(curr[1], curr[0]);
        alpha2[i] = 2 * asin(sin(elevation[i] / 2) / sin(gamma));
        eta = acos((sin(alpha2[i] / 2) - sin(gamma) * sin(elevation[i] / 2)) / (cos(elevation[i] / 2) * cos(gamma)));
        alpha1[i] = PI - eta - azimuth[i];
        printf("elevation %f , azimuth %f , alpha1 %f , alpha2 %f \n",elevation[i],azimuth[i],alpha1[i],alpha2[i]) ;
    }

    return 0;
}



// Matrix multiplication
void matmul(double a[3][3], double b[3][3], double result[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

// Matrix-vector multiplication
void matvecmul(const double mat[3][3], const double vec[3], double result[3]) {
    for (int i = 0; i < 3; i++) {
        result[i] = 0;
        for (int j = 0; j < 3; j++) {
            result[i] += mat[i][j] * vec[j];
        }
    }
}

// Cross product
void cross(double a[3], double b[3], double result[3]) {
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

// Dot product
double dot(double a[3], double b[3]) {
    double result = 0;
    for (int i = 0; i < 3; i++) {
        result += a[i] * b[i];
    }
    return result;
}

// Rotation about y-axis
void roty(double angle, double result[3][3]) {
    result[0][0] = cos(angle);
    result[0][1] = 0;
    result[0][2] = sin(angle);
    result[1][0] = 0;
    result[1][1] = 1;
    result[1][2] = 0;
    result[2][0] = -sin(angle);
    result[2][1] = 0;
    result[2][2] = cos(angle);
}

// Rotation about z-axis
void rotz(double angle, double result[3][3]) {
    result[0][0] = cos(angle);
    result[0][1] = -sin(angle);
    result[0][2] = 0;
    result[1][0] = sin(angle);
    result[1][1] = cos(angle);
    result[1][2] = 0;
    result[2][0] = 0;
    result[2][1] = 0;
    result[2][2] = 1;
}

// Axis-angle to rotation matrix
void axang2rotm(double axis[3], double angle, double result[3][3]) {
    double c = cos(angle);
    double s = sin(angle);
    double t = 1 - c;
    double x = axis[0], y = axis[1], z = axis[2];

    result[0][0] = t*x*x + c;
    result[0][1] = t*x*y - s*z;
    result[0][2] = t*x*z + s*y;
    result[1][0] = t*x*y + s*z;
    result[1][1] = t*y*y + c;
    result[1][2] = t*y*z - s*x;
    result[2][0] = t*x*z - s*y;
    result[2][1] = t*y*z + s*x;
    result[2][2] = t*z*z + c;
}
