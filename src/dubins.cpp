// Copyright (c) 2008-2014, Andrew Walker
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef DUBINS_CPP
#define DUBINS_CPP

#include "dubins.h"
#define _USE_MATH_DEFINES // for C++
#include <math.h>
#include <assert.h>

#define EPSILON (10e-10)

#define LSL (0)
#define LSR (1)
#define RSL (2)
#define RSR (3)
#define RLR (4)
#define LRL (5)

// The three segment types a path can be made up of
#define L_SEG (0)
#define S_SEG (1)
#define R_SEG (2)

namespace HybridAStar{

// The segment types for each of the Path types
const int DIRDATA[][3] = {
    { L_SEG, S_SEG, L_SEG },
    { L_SEG, S_SEG, R_SEG },
    { R_SEG, S_SEG, L_SEG },
    { R_SEG, S_SEG, R_SEG },
    { R_SEG, L_SEG, R_SEG },
    { L_SEG, R_SEG, L_SEG }
};

DubinsWord dubins_words[] = {
    dubins_LSL,
    dubins_LSR,
    dubins_RSL,
    dubins_RSR,
    dubins_RLR,
    dubins_LRL,
};

#define UNPACK_INPUTS(alpha, beta) \
    float sa = sin(alpha);         \
    float sb = sin(beta);          \
    float ca = cos(alpha);         \
    float cb = cos(beta);          \
    float c_ab = cos(alpha - beta);

#define PACK_OUTPUTS(outputs)       \
    outputs[0]  = t;                \
    outputs[1]  = p;                \
    outputs[2]  = q;

/**
 * Floating point modulus suitable for rings
 *
 * fmod doesn't behave correctly for angular quantities, this function does
 */
float fmodr(float x, float y)
{
    return x - y*floor(x/y);
}

float mod2pi(float theta)
{
    return fmodr( theta, 2 * M_PI );
}

int dubins_init_normalised(float alpha, float beta, float d, DubinsPath *path)
{
    float best_cost = INFINITY;
    int    best_word;
    int    i;

    best_word = -1;
    for( i = 0; i < 6; i++ ) {
        float params[3];
        int err = dubins_words[i](alpha, beta, d, params);
        if(err == EDUBOK) {
            float cost = params[0] + params[1] + params[2];
            if(cost < best_cost) {
                best_word = i;
                best_cost = cost;
                path->param[0] = params[0];
                path->param[1] = params[1];
                path->param[2] = params[2];
                path->type = i;
            }
        }
    }

    if(best_word == -1) {
        return EDUBNOPATH;
    }
    path->type = best_word;
    return EDUBOK;
}

int dubins_init(float q0[3], float q1[3], float rho, DubinsPath *path)
{
    int i;
    float dx = q1[0] - q0[0];
    float dy = q1[1] - q0[1];
    float D = sqrt(dx * dx + dy * dy);
    float d = D / rho;
    if( rho <= 0. ) {
        return EDUBBADRHO;
    }
    float theta = mod2pi(atan2(dy, dx));
    float alpha = mod2pi(q0[2] - theta);
    float beta = mod2pi(q1[2] - theta);
    for( i = 0; i < 3; i ++ ) {
        path->qi[i] = q0[i];
    }
    path->rho = rho;

    return dubins_init_normalised( alpha, beta, d, path );
}

int dubins_LSL(float alpha, float beta, float d, float *outputs)
{
    UNPACK_INPUTS(alpha, beta);
    float tmp0 = d + sa - sb;
    float p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb));
    if( p_squared < 0 ) {
        return EDUBNOPATH;
    }
    float tmp1 = atan2((cb - ca), tmp0);
    float t = mod2pi(-alpha + tmp1);
    float p = sqrt(p_squared);
    float q = mod2pi(beta - tmp1);
    PACK_OUTPUTS(outputs);
    return EDUBOK;
}

int dubins_RSR(float alpha, float beta, float d, float *outputs)
{
    UNPACK_INPUTS(alpha, beta);
    float tmp0 = d - sa + sb;
    float p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sb - sa));
    if( p_squared < 0 ) {
        return EDUBNOPATH;
    }
    float tmp1 = atan2((ca - cb), tmp0);
    float t = mod2pi(alpha - tmp1);
    float p = sqrt(p_squared);
    float q = mod2pi(-beta + tmp1);
    PACK_OUTPUTS(outputs);
    return EDUBOK;
}

int dubins_LSR(float alpha, float beta, float d, float *outputs)
{
    UNPACK_INPUTS(alpha, beta);
    float p_squared = -2 + (d * d) + (2 * c_ab) + (2 * d * (sa + sb));
    if( p_squared < 0 ) {
        return EDUBNOPATH;
    }
    float p = sqrt(p_squared);
    float tmp2 = atan2((-ca - cb), (d + sa + sb)) - atan2(-2.0, p);
    float t = mod2pi(-alpha + tmp2);
    float q = mod2pi(-mod2pi(beta) + tmp2);
    PACK_OUTPUTS(outputs);
    return EDUBOK;
}

int dubins_RSL(float alpha, float beta, float d, float *outputs)
{
    UNPACK_INPUTS(alpha, beta);
    float p_squared = (d * d) - 2 + (2 * c_ab) - (2 * d * (sa + sb));
    if( p_squared< 0 ) {
        return EDUBNOPATH;
    }
    float p = sqrt(p_squared);
    float tmp2 = atan2((ca + cb), (d - sa - sb)) - atan2(2.0, p);
    float t = mod2pi(alpha - tmp2);
    float q = mod2pi(beta - tmp2);
    PACK_OUTPUTS(outputs);
    return EDUBOK;
}

int dubins_RLR(float alpha, float beta, float d, float *outputs)
{
    UNPACK_INPUTS(alpha, beta);
    float tmp_rlr = (6. - d * d + 2 * c_ab + 2 * d * (sa - sb)) / 8.;
    if( fabs(tmp_rlr) > 1) {
        return EDUBNOPATH;
    }
    float p = mod2pi(2 * M_PI - acos(tmp_rlr));
    float t = mod2pi(alpha - atan2(ca - cb, d - sa + sb) + mod2pi(p / 2.));
    float q = mod2pi(alpha - beta - t + mod2pi(p));
    PACK_OUTPUTS( outputs );
    return EDUBOK;
}

int dubins_LRL(float alpha, float beta, float d, float *outputs)
{
    UNPACK_INPUTS(alpha, beta);
    float tmp_lrl = (6. - d * d + 2 * c_ab + 2 * d * (-sa + sb)) / 8.;
    if( fabs(tmp_lrl) > 1) {
        return EDUBNOPATH;
    }
    float p = mod2pi(2 * M_PI - acos(tmp_lrl));
    float t = mod2pi(-alpha - atan2(ca - cb, d + sa - sb) + p / 2.);
    float q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p));
    PACK_OUTPUTS( outputs );
    return EDUBOK;
}

float dubins_path_length(DubinsPath *path)
{
    float length = 0.;
    length += path->param[0];
    length += path->param[1];
    length += path->param[2];
    length = length * path->rho;
    return length;
}

int dubins_path_type( DubinsPath* path ) {
    return path->type;
}

void dubins_segment(float t, float qi[3], float qt[3], int type)
{
    assert( type == L_SEG || type == S_SEG || type == R_SEG );

    if( type == L_SEG ) {
        qt[0] = qi[0] + sin(qi[2]+t) - sin(qi[2]);
        qt[1] = qi[1] - cos(qi[2]+t) + cos(qi[2]);
        qt[2] = qi[2] + t;
    }
    else if( type == R_SEG ) {
        qt[0] = qi[0] - sin(qi[2]-t) + sin(qi[2]);
        qt[1] = qi[1] + cos(qi[2]-t) - cos(qi[2]);
        qt[2] = qi[2] - t;
    }
    else if( type == S_SEG ) {
        qt[0] = qi[0] + cos(qi[2]) * t;
        qt[1] = qi[1] + sin(qi[2]) * t;
        qt[2] = qi[2];
    }
}

int dubins_path_sample(DubinsPath *path, float t, float q[3])
{
    if( t < 0 || t >= dubins_path_length(path) ) {
        // error, parameter out of bounds
        return EDUBPARAM;
    }

    // tprime is the normalised variant of the parameter t
    float tprime = t / path->rho;

    // In order to take rho != 1 into account this function needs to be more complex
    // than it would be otherwise. The transformation is done in five stages.
    //
    // 1. translate the components of the initial configuration to the origin
    // 2. generate the target configuration
    // 3. transform the target configuration
    //      scale the target configuration
    //      translate the target configration back to the original starting point
    //      normalise the target configurations angular component

    // The translated initial configuration
    float qi[3] = {0, 0, path->qi[2]};

    // Generate the target configuration
    const int* types = DIRDATA[path->type];
    float p1 = path->param[0];
    float p2 = path->param[1];
    float q1[3]; // end-of segment 1
    float q2[3]; // end-of segment 2
    dubins_segment( p1,      qi,    q1, types[0] );
    dubins_segment( p2,      q1,    q2, types[1] );
    if( tprime < p1 ) {
        dubins_segment( tprime, qi, q, types[0] );
    }
    else if( tprime < (p1+p2) ) {
        dubins_segment( tprime-p1, q1, q,  types[1] );
    }
    else {
        dubins_segment( tprime-p1-p2, q2, q,  types[2] );
    }

    // scale the target configuration, translate back to the original starting point
    q[0] = q[0] * path->rho + path->qi[0];
    q[1] = q[1] * path->rho + path->qi[1];
    q[2] = mod2pi(q[2]);

    return 0;
}

int dubins_path_sample_many(DubinsPath *path, DubinsPathSamplingCallback cb, float stepSize, void *user_data)
{
    float x = 0.0;
    float length = dubins_path_length(path);
    while( x <  length ) {
        float q[3];
        dubins_path_sample( path, x, q );
        int retcode = cb(q, x, user_data);
        if( retcode != 0 ) {
            return retcode;
        }
        x += stepSize;
    }
    return 0;
}

int dubins_path_endpoint(DubinsPath *path, float q[3])
{
    return dubins_path_sample( path, dubins_path_length(path) - EPSILON, q );
}

int dubins_extract_subpath(DubinsPath *path, float t, DubinsPath *newpath)
{
    // calculate the true parameter
    float tprime = t / path->rho;

    // copy most of the data
    newpath->qi[0] = path->qi[0];
    newpath->qi[1] = path->qi[1];
    newpath->qi[2] = path->qi[2];
    newpath->rho   = path->rho;
    newpath->type  = path->type;

    // fix the parameters
    newpath->param[0] = fmin( path->param[0], tprime );
    newpath->param[1] = fmin( path->param[1], tprime - newpath->param[0]);
    newpath->param[2] = fmin( path->param[2], tprime - newpath->param[0] - newpath->param[1]);
    return 0;
}
}
#endif
