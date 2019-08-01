#include "fusion.h"
#include "math.h"
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion

float deltat = 1.0f / 50.0f; // integration interval for both filter schemes
float beta = 0.1f;           // compute beta
float zeta = 0.0f;           // compute beta
#define Kp 2.0f * 5.0f       // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f
float eInt[3] = {0.0f, 0.0f, 0.0f}; // vector to hold integral error for Mahony method


float b_x = 1, b_z = 0; // reference direction of flux in earth frame
float w_bx = 0, w_by = 0, w_bz = 0; // estimate gyroscope biases error

float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float *qt, float deltat_time)
// {
//     float recipNorm;
//     float s0, s1, s2, s3;
//     float qDot1, qDot2, qDot3, qDot4;
//     float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

//     // Convert gyroscope degrees/sec to radians/sec
//     // gx *= 0.0174533f;
//     // gy *= 0.0174533f;
//     // gz *= 0.0174533f;

//     // Rate of change of quaternion from gyroscope
//     qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
//     qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
//     qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
//     qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

//     // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
//     if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
//     {

//         // Normalise accelerometer measurement
//         recipNorm = invSqrt(ax * ax + ay * ay + az * az);
//         ax *= recipNorm;
//         ay *= recipNorm;
//         az *= recipNorm;

//         // Auxiliary variables to avoid repeated arithmetic
//         _2q0 = 2.0f * q[0];
//         _2q1 = 2.0f * q[1];
//         _2q2 = 2.0f * q[2];
//         _2q3 = 2.0f * q[3];
//         _4q0 = 4.0f * q[0];
//         _4q1 = 4.0f * q[1];
//         _4q2 = 4.0f * q[2];
//         _8q1 = 8.0f * q[1];
//         _8q2 = 8.0f * q[2];
//         q0q0 = q[0] * q[0];
//         q1q1 = q[1] * q[1];
//         q2q2 = q[2] * q[2];
//         q3q3 = q[3] * q[3];

//         // Gradient decent algorithm corrective step
//         s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
//         s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
//         s2 = 4.0f * q0q0 * q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
//         s3 = 4.0f * q1q1 * q[3] - _2q1 * ax + 4.0f * q2q2 * q[3] - _2q2 * ay;
//         recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
//         s0 *= recipNorm;
//         s1 *= recipNorm;
//         s2 *= recipNorm;
//         s3 *= recipNorm;

//         // Apply feedback step
//         qDot1 -= beta * s0;
//         qDot2 -= beta * s1;
//         qDot3 -= beta * s2;
//         qDot4 -= beta * s3;
//     }

//     // Integrate rate of change of quaternion to yield quaternion
//     q[0] += qDot1 * deltat_time;
//     q[1] += qDot2 * deltat_time;
//     q[2] += qDot3 * deltat_time;
//     q[3] += qDot4 * deltat_time;

//     // Normalise quaternion
//     recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
//     q[0] *= recipNorm;
//     q[1] *= recipNorm;
//     q[2] *= recipNorm;
//     q[3] *= recipNorm;
//     qt[0] = q[0];
//     qt[1] = q[1];
//     qt[2] = q[2];
//     qt[3] = q[3];
// }

// paper 原版
void MadgwickQuaternionUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z, float *qt, float deltat_time)
{
    // local system variables
    float norm;                                                                 // vector norm
    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4;       // quaternion rate from gyroscopes elements
    float f_1, f_2, f_3, f_4, f_5, f_6;                                         // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33,                   // objective function Jacobian elements
        J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;                   // estimated direction of the gyroscope error
    float w_err_x, w_err_y, w_err_z;                                            // estimated direction of the gyroscope error (angular)
    float h_x, h_y, h_z;                                                        // computed flux in the earth frame
    // axulirary variables to avoid reapeated calcualtions
    float halfSEq_1 = 0.5f * q[0];
    float halfSEq_2 = 0.5f * q[1];
    float halfSEq_3 = 0.5f * q[2];
    float halfSEq_4 = 0.5f * q[3];
    float twoSEq_1 = 2.0f * q[0];
    float twoSEq_2 = 2.0f * q[1];
    float twoSEq_3 = 2.0f * q[2];
    float twoSEq_4 = 2.0f * q[3];
    float twob_x = 2.0f * b_x;
    float twob_z = 2.0f * b_z;
    float twob_xSEq_1 = 2.0f * b_x * q[0];
    float twob_xSEq_2 = 2.0f * b_x * q[1];
    float twob_xSEq_3 = 2.0f * b_x * q[2];
    float twob_xSEq_4 = 2.0f * b_x * q[3];
    float twob_zSEq_1 = 2.0f * b_z * q[0];
    float twob_zSEq_2 = 2.0f * b_z * q[1];
    float twob_zSEq_3 = 2.0f * b_z * q[2];
    float twob_zSEq_4 = 2.0f * b_z * q[3];
    float SEq_1SEq_2;
    float SEq_1SEq_3 = q[0] * q[2];
    float SEq_1SEq_4;
    float SEq_2SEq_3;
    float SEq_2SEq_4 = q[1] * q[3];
    float SEq_3SEq_4;
    float twom_x = 2.0f * m_x;
    float twom_y = 2.0f * m_y;
    float twom_z = 2.0f * m_z;
    // normalise the accelerometer measurement
    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x /= norm;
    a_y /= norm;
    a_z /= norm;
    // normalise the magnetometer measurement
    norm = sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
    m_x /= norm;
    m_y /= norm;
    m_z /= norm;
    // compute the objective function and Jacobian
    f_1 = twoSEq_2 * q[3] - twoSEq_1 * q[2] - a_x;
    f_2 = twoSEq_1 * q[1] + twoSEq_3 * q[3] - a_y;
    f_3 = 1.0f - twoSEq_2 * q[1] - twoSEq_3 * q[2] - a_z;
    f_4 = twob_x * (0.5f - q[2] * q[2] - q[3] * q[3]) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
    f_5 = twob_x * (q[1] * q[2] - q[0] * q[3]) + twob_z * (q[0] * q[1] + q[2] * q[3]) - m_y;
    f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - q[1] * q[1] - q[2] * q[2]) - m_z;
    J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * q[3];
    J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
    J_14or21 = twoSEq_2;
    J_32 = 2.0f * J_14or21; // negated in matrix multiplication
    J_33 = 2.0f * J_11or24; // negated in matrix multiplication
    J_41 = twob_zSEq_3;     // negated in matrix multiplication
    J_42 = twob_zSEq_4;
    J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1; // negated in matrix multiplication
    J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
    J_51 = twob_xSEq_4 - twob_zSEq_2;        // negated in matrix multiplication
    J_52 = twob_xSEq_3 + twob_zSEq_1;
    J_53 = twob_xSEq_2 + twob_zSEq_4;
    J_54 = twob_xSEq_1 - twob_zSEq_3; // negated in matrix multiplication
    J_61 = twob_xSEq_3;
    J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
    J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
    J_64 = twob_xSEq_2;
    // compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;
    // normalise the gradient to estimate direction of the gyroscope error
    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 = SEqHatDot_1 / norm;
    SEqHatDot_2 = SEqHatDot_2 / norm;
    SEqHatDot_3 = SEqHatDot_3 / norm;
    SEqHatDot_4 = SEqHatDot_4 / norm;
    // compute angular estimated direction of the gyroscope error
    w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
    w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
    w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
    // compute and remove the gyroscope baises
    w_bx += w_err_x * deltat_time * zeta;
    w_by += w_err_y * deltat_time * zeta;
    w_bz += w_err_z * deltat_time * zeta;
    w_x -= w_bx;
    w_y -= w_by;
    w_z -= w_bz;
    // compute the quaternion rate measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
    // compute then integrate the estimated quaternion rate
    q[0] += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat_time;
    q[1] += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat_time;
    q[2] += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat_time;
    q[3] += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat_time;
    // normalise quaternion
    norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;
    // compute flux in the earth frame
    SEq_1SEq_2 = q[0] * q[1]; // recompute axulirary variables
    SEq_1SEq_3 = q[0] * q[2];
    SEq_1SEq_4 = q[0] * q[3];
    SEq_3SEq_4 = q[2] * q[3];
    SEq_2SEq_3 = q[1] * q[2];
    SEq_2SEq_4 = q[1] * q[3];
    h_x = twom_x * (0.5f - q[2] * q[2] - q[3] * q[3]) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
    h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - q[1] * q[1] - q[3] * q[3]) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
    h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - q[1] * q[1] - q[2] * q[2]);
    // normalise the flux vector to have only components in the x and z
    b_x = sqrt((h_x * h_x) + (h_y * h_y));
    b_z = h_z;

    qt[0] = q[0];
    qt[1] = q[1];
    qt[2] = q[2];
    qt[3] = q[3];
}

// void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz,  float *qt, float deltat_time)
//     {
//         float q[1] = q[0], q[2] = q[1], q[3] = q[2], q4 = q[3];   // short name local variable for readability
//         float norm;
//         float hx, hy, bx, bz;
//         float vx, vy, vz, wx, wy, wz;
//         float ex, ey, ez;
//         float pa, pb, pc;

//         // Auxiliary variables to avoid repeated arithmetic
//         float q1q1 = q[1] * q[1];
//         float q1q2 = q[1] * q[2];
//         float q1q3 = q[1] * q[3];
//         float q1q4 = q[1] * q4;
//         float q2q2 = q[2] * q[2];
//         float q2q3 = q[2] * q[3];
//         float q2q4 = q[2] * q4;
//         float q3q3 = q[3] * q[3];
//         float q3q4 = q[3] * q4;
//         float q4q4 = q4 * q4;

//         // Normalise accelerometer measurement
//         norm = sqrt(ax * ax + ay * ay + az * az);
//         if (norm == 0.0f) return; // handle NaN
//         norm = 1.0f / norm;        // use reciprocal for division
//         ax *= norm;
//         ay *= norm;
//         az *= norm;

//         // Normalise magnetometer measurement
//         norm = sqrt(mx * mx + my * my + mz * mz);
//         if (norm == 0.0f) return; // handle NaN
//         norm = 1.0f / norm;        // use reciprocal for division
//         mx *= norm;
//         my *= norm;
//         mz *= norm;

//         // Reference direction of Earth's magnetic field
//         hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
//         hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
//         bx = sqrt((hx * hx) + (hy * hy));
//         bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

//         // Estimated direction of gravity and magnetic field
//         vx = 2.0f * (q2q4 - q1q3);
//         vy = 2.0f * (q1q2 + q3q4);
//         vz = q1q1 - q2q2 - q3q3 + q4q4;
//         wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
//         wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
//         wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

//         // Error is cross product between estimated direction and measured direction of gravity
//         ex = (ay * vz - az * vy) + (my * wz - mz * wy);
//         ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
//         ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
//         if (Ki > 0.0f)
//         {
//             eInt[0] += ex;      // accumulate integral error
//             eInt[1] += ey;
//             eInt[2] += ez;
//         }
//         else
//         {
//             eInt[0] = 0.0f;     // prevent integral wind up
//             eInt[1] = 0.0f;
//             eInt[2] = 0.0f;
//         }

//         // Apply feedback terms
//         gx = gx + Kp * ex + Ki * eInt[0];
//         gy = gy + Kp * ey + Ki * eInt[1];
//         gz = gz + Kp * ez + Ki * eInt[2];

//         // Integrate rate of change of quaternion
//         pa = q[2];
//         pb = q[3];
//         pc = q4;
//         q[1] = q[1] + (-q[2] * gx - q[3] * gy - q4 * gz) * (0.5f * deltat_time);
//         q[2] = pa + (q[1] * gx + pb * gz - pc * gy) * (0.5f * deltat_time);
//         q[3] = pb + (q[1] * gy - pa * gz + pc * gx) * (0.5f * deltat_time);
//         q4 = pc + (q[1] * gz + pa * gy - pb * gx) * (0.5f * deltat_time);

//         // Normalise quaternion
//         norm = sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3] + q4 * q4);
//         norm = 1.0f / norm;
//         q[0] = q[1] * norm;
//         q[1] = q[2] * norm;
//         q[2] = q[3] * norm;
//         q[3] = q4 * norm;
//         qt[0] = q[0];
//         qt[1] = q[1];
//         qt[2] = q[2];
//         qt[3] = q[3];

//     }