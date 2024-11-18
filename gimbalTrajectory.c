#include <math.h>
#include <stdio.h>

#define PI 3.14159265
#define DEG2RAD(deg) ((deg) * PI / 180.0)
#define RAD2DEG(rad) ((rad) * 180.0 / PI)
#define NUMOFSTEPS 10
#define GAMMA  (DEG2RAD(20))

// Assuming you have implemented these functions
void roty(float angle, float matrix[3][3]);
void rotz(float angle, float matrix[3][3]);
void cross(float a[3], float b[3], float result[3]);
float dot(float a[3], float b[3]);
void axang2rotm(float axis[3], float angle, float matrix[3][3]);
void matvecmul(const float mat[3][3], const float vec[3], float result[3]);
void matmul(float a[3][3], float b[3][3], float result[3][3]) ;

void getSteps(const float elevation1, const float azimuth1, const float elevation2, const float azimuth2,
float alpha2[NUMOFSTEPS], float alpha1[NUMOFSTEPS]);

int main() 
{
    float elevation1 = DEG2RAD(5);
    float azimuth1 = DEG2RAD(5);
    float elevation2 = DEG2RAD(40);
    float azimuth2 = DEG2RAD(60);

    // gimbal_trajectory(elevation1=np.deg2rad(5), azimuth1=np.deg2rad(5), elevation2=np.deg2rad(40), azimuth2=np.deg2rad(60))
    // gimbal_trajectory(elevation1=np.deg2rad(40), azimuth1=np.deg2rad(60), elevation2=np.deg2rad(40), azimuth2=np.deg2rad(-30), figax=figax)
    // gimbal_trajectory(elevation1=np.deg2rad(40), azimuth1=np.deg2rad(-30), elevation2=np.deg2rad(5), azimuth2=np.deg2rad(5), figax=figax)

    float alpha1[NUMOFSTEPS]={0};
    float alpha2[NUMOFSTEPS]={0};
    getSteps(elevation1, azimuth1, elevation2, azimuth2, alpha1, alpha2);
}

void alpha12toAzEl(const float alpha1, const float alpha2, float* azimuth, float* elevation)
{
    *elevation = 2 * asin(sin(alpha2 / 2) * sin(GAMMA));
    float eta = acos((sin(alpha2 / 2) - sin(GAMMA) * sin(*elevation / 2)) / (cos(*elevation / 2) * cos(GAMMA)));
    *azimuth = PI - eta - alpha1;
}

void azElToAlpha12(const float azimuth, const float elevation, float *alpha1, float *alpha2 )
{
    *alpha2 = 2 * asin(sin(elevation / 2) / sin(GAMMA));
    float eta = acos((sin(*alpha2 / 2) - sin(GAMMA) * sin(elevation / 2)) / (cos(elevation / 2) * cos(GAMMA)));
    *alpha1 = PI - eta - azimuth;
}

void getSteps(const float elevation1, const float azimuth1, const float elevation2, const float azimuth2,
            float alpha1[NUMOFSTEPS], float alpha2[NUMOFSTEPS])
{
    float rotElev1[3][3], rotAz1[3][3], rotElev2[3][3], rotAz2[3][3];
    float rot1[3][3], rot2[3][3];
    float rinit[3] = {0, 0, 1};
    float r1[3], r2[3];
    float r1r2axis[3], theta;

    float dtheta, rotdtheta[3][3];
    float elevation[NUMOFSTEPS], azimuth[NUMOFSTEPS],  eta;

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
    float norm = sqrt(dot(r1r2axis, r1r2axis));
    for (int i = 0; i < 3; i++) {
        r1r2axis[i] /= norm;
    }

    theta = acos(dot(r1, r2));
    dtheta = theta / NUMOFSTEPS;

    axang2rotm(r1r2axis, dtheta, rotdtheta);

    float curr[3] = {r1[0], r1[1], r1[2]};
    for (int i = 0; i < NUMOFSTEPS; i++) 
    {
        float ncurr[3]={curr[0],curr[1],curr[2]};

        matvecmul(rotdtheta, ncurr, curr);
        elevation[i] = acos(dot(curr, (float[]){0, 0, 1}));
        azimuth[i] = atan2(curr[1], curr[0]);
        alpha2[i] = 2 * asin(sin(elevation[i] / 2) / sin(GAMMA));
        eta = acos((sin(alpha2[i] / 2) - sin(GAMMA) * sin(elevation[i] / 2)) / (cos(elevation[i] / 2) * cos(GAMMA)));
        alpha1[i] = PI - eta - azimuth[i];
        printf("elevation %f , azimuth %f , alpha1_deg %f , alpha2_deg %f , alpha1 %f , alpha2 %f \n",elevation[i],azimuth[i],alpha1[i]/PI*180,alpha2[i]/PI*180,alpha1[i],alpha2[i]) ;
    }
}



// Matrix multiplication
void matmul(float a[3][3], float b[3][3], float result[3][3]) 
{
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
void matvecmul(const float mat[3][3], const float vec[3], float result[3]) {
    for (int i = 0; i < 3; i++) {
        result[i] = 0;
        for (int j = 0; j < 3; j++) {
            result[i] += mat[i][j] * vec[j];
        }
    }
}

// Cross product
void cross(float a[3], float b[3], float result[3]) {
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

// Dot product
float dot(float a[3], float b[3]) {
    float result = 0;
    for (int i = 0; i < 3; i++) {
        result += a[i] * b[i];
    }
    return result;
}

// Rotation about y-axis
void roty(float angle, float result[3][3]) {
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
void rotz(float angle, float result[3][3]) {
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
void axang2rotm(float axis[3], float angle, float result[3][3]) {
    float c = cos(angle);
    float s = sin(angle);
    float t = 1 - c;
    float x = axis[0], y = axis[1], z = axis[2];

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
