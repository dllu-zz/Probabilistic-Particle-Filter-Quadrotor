/* quaternion.h
This file is modified, by Daniel L. Lu, from quaternion.h from the Irrlicht Engine.

Here is the original copyright message and license information pertaining hereto:

==================================================================================

  Copyright (C) 2002-2012 Nikolaus Gebhardt

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.

  Please note that the Irrlicht Engine is based in part on the work of the
  Independent JPEG Group, the zlib and the libPng. This means that if you use
  the Irrlicht Engine in your product, you must acknowledge somewhere in your
  documentation that you've used the IJG code. It would also be nice to mention
  that you use the Irrlicht Engine, the zlib and libPng. See the README files
  in the jpeglib, the zlib and libPng for further informations.
*/

#ifndef QUATERNION_H
#define QUATERNION_H

class quaternion {
public:
    //! Default
    quaternion(): X(0.0), Y(0.0), Z(0.0), W(1.0) {}

    //! Constructor
    quaternion(double x, double y, double z, double w): X(x), Y(y), Z(z), W(w) {}

    //! Constructor which converts Euler angles (radians) to a quaternion
    quaternion(double x, double y, double z);

    //! Equalilty operator
    bool operator==(const quaternion& other) const;

    //! inequality operator
    bool operator!=(const quaternion& other) const;

    //! Assignment operator
    inline quaternion& operator=(const quaternion& other);

    //! Add operator
    quaternion operator+(const quaternion& other) const;

    //! Multiplication operator
    quaternion operator*(const quaternion& other) const;

    //! Multiplication operator with scalar
    quaternion operator*(double s) const;

    //! Multiplication operator with scalar
    quaternion& operator*=(double s);

    //! Multiplication operator
    quaternion& operator*=(const quaternion& other);

    //! Calculates the dot product
    inline double dotProduct(const quaternion& other) const;

    //! Sets new quaternion
    inline quaternion& set(double x, double y, double z, double w);

    //! Sets new quaternion based on euler angles (radians)
    inline quaternion& set(double x, double y, double z);

    //! Sets new quaternion from other quaternion
    inline quaternion& set(const core::quaternion& quat);

    //! returns if this quaternion equals the other one, taking floating point rounding errors into account
    inline bool equals(const quaternion& other,
            const double tolerance) const;

    //! Normalizes the quaternion
    inline quaternion& normalize();

    //! Inverts this quaternion
    quaternion& makeInverse();

    //! Set this quaternion to the linear interpolation between two quaternions
    /** \param q1 First quaternion to be interpolated.
    \param q2 Second quaternion to be interpolated.
    \param time Progress of interpolation. For time=0 the result is
    q1, for time=1 the result is q2. Otherwise interpolation
    between q1 and q2.
    */
    quaternion& lerp(quaternion q1, quaternion q2, double time);

    //! Set this quaternion to the result of the spherical interpolation between two quaternions
    /** \param q1 First quaternion to be interpolated.
    \param q2 Second quaternion to be interpolated.
    \param time Progress of interpolation. For time=0 the result is
    q1, for time=1 the result is q2. Otherwise interpolation
    between q1 and q2.
    \param threshold To avoid inaccuracies at the end (time=1) the
    interpolation switches to linear interpolation at some point.
    This value defines how much of the remaining interpolation will
    be calculated with lerp. Everything from 1-threshold up will be
    linear interpolation.
    */
    quaternion& slerp(quaternion q1, quaternion q2,
            double time, double threshold=.05f);

    //! Create quaternion from rotation angle and rotation axis.
    /** Axis must be unit length.
    The quaternion representing the rotation is
    q = cos(A/2)+sin(A/2)*(x*i+y*j+z*k).
    \param angle Rotation Angle in radians.
    \param axis Rotation axis. */
    quaternion& fromAngleAxis (double angle, const vector3df& axis);

    //! Fills an angle (radians) around an axis (unit vector)
    void toAngleAxis (double &angle, core::vector3df& axis) const;

    //! Output this quaternion to an euler angle (radians)
    void toEuler(vector3df& euler) const;

    //! Set quaternion to identity
    quaternion& makeIdentity();

    //! Set quaternion to represent a rotation from one vector to another.
    quaternion& rotationFromTo(const vector3df& from, const vector3df& to);

    //! Quaternion elements.
    double X; // vectorial (imaginary) part
    double Y;
    double Z;
    double W; // real part
};

// Constructor which converts euler angles to a quaternion
inline quaternion::quaternion(double x, double y, double z) {
    set(x,y,z);
}

// equal operator
inline bool quaternion::operator==(const quaternion& other) const {
    return ((X == other.X) &&
        (Y == other.Y) &&
        (Z == other.Z) &&
        (W == other.W));
}

// inequality operator
inline bool quaternion::operator!=(const quaternion& other) const {
    return !(*this == other);
}

// assignment operator
inline quaternion& quaternion::operator=(const quaternion& other) {
    X = other.X;
    Y = other.Y;
    Z = other.Z;
    W = other.W;
    return *this;
}

// multiplication operator
inline quaternion quaternion::operator*(const quaternion& other) const {
    quaternion tmp;

    tmp.W = (other.W * W) - (other.X * X) - (other.Y * Y) - (other.Z * Z);
    tmp.X = (other.W * X) + (other.X * W) + (other.Y * Z) - (other.Z * Y);
    tmp.Y = (other.W * Y) + (other.Y * W) + (other.Z * X) - (other.X * Z);
    tmp.Z = (other.W * Z) + (other.Z * W) + (other.X * Y) - (other.Y * X);

    return tmp;
}

// multiplication operator
inline quaternion quaternion::operator*(double s) const {
    return quaternion(s*X, s*Y, s*Z, s*W);
}

// multiplication operator
inline quaternion& quaternion::operator*=(double s) {
    X*=s;
    Y*=s;
    Z*=s;
    W*=s;
    return *this;
}

// multiplication operator
inline quaternion& quaternion::operator*=(const quaternion& other) {
    return (*this = other * (*this));
}

// add operator
inline quaternion quaternion::operator+(const quaternion& b) const {
    return quaternion(X+b.X, Y+b.Y, Z+b.Z, W+b.W);
}

// Inverts this quaternion
inline quaternion& quaternion::makeInverse() {
    X = -X; Y = -Y; Z = -Z;
    return *this;
}


// sets new quaternion
inline quaternion& quaternion::set(double x, double y, double z, double w) {
    X = x;
    Y = y;
    Z = z;
    W = w;
    return *this;
}

// sets new quaternion based on euler angles
inline quaternion& quaternion::set(double x, double y, double z) {
    long double angle;

    angle = x * 0.5;
    const long double sr = sin(angle);
    const long double cr = cos(angle);

    angle = y * 0.5;
    const long double sp = sin(angle);
    const long double cp = cos(angle);

    angle = z * 0.5;
    const long double sy = sin(angle);
    const long double cy = cos(angle);

    const long double cpcy = cp * cy;
    const long double spcy = sp * cy;
    const long double cpsy = cp * sy;
    const long double spsy = sp * sy;

    X = (double)(sr * cpcy - cr * spsy);
    Y = (double)(cr * spcy + sr * cpsy);
    Z = (double)(cr * cpsy - sr * spcy);
    W = (double)(cr * cpcy + sr * spsy);

    return normalize();
}

// sets new quaternion based on other quaternion
inline quaternion& quaternion::set(const core::quaternion& quat) {
    return (*this=quat);
}

//! returns if this quaternion equals the other one, taking floating point rounding errors into account
inline bool quaternion::equals(const quaternion& other, const double tolerance) const {
    return core::equals(X, other.X, tolerance) &&
        core::equals(Y, other.Y, tolerance) &&
        core::equals(Z, other.Z, tolerance) &&
        core::equals(W, other.W, tolerance);
}

// normalizes the quaternion
inline quaternion& quaternion::normalize() {
    const double n = X*X + Y*Y + Z*Z + W*W;

    if (n == 1)
        return *this;

    return (*this *= 1.0/sqrt(n));
}

// set this quaternion to the result of the linear interpolation between two quaternions
inline quaternion& quaternion::lerp(quaternion q1, quaternion q2, double time) {
    const double scale = 1.0f - time;
    return (*this = (q1*scale) + (q2*time));
}

// set this quaternion to the result of the interpolation between two quaternions
inline quaternion& quaternion::slerp(quaternion q1, quaternion q2, double time, double threshold) {
    double angle = q1.dotProduct(q2);

    // make sure we use the short rotation
    if (angle < 0.0f)
    {
        q1 *= -1.0f;
        angle *= -1.0f;
    }

    if (angle <= (1-threshold)) // spherical interpolation
    {
        const double theta = acosf(angle);
        const double invsintheta = reciprocal(sinf(theta));
        const double scale = sinf(theta * (1.0f-time)) * invsintheta;
        const double invscale = sinf(theta * time) * invsintheta;
        return (*this = (q1*scale) + (q2*invscale));
    }
    else // linear interploation
        return lerp(q1,q2,time);
}


// calculates the dot product
inline double quaternion::dotProduct(const quaternion& q2) const {
    return (X * q2.X) + (Y * q2.Y) + (Z * q2.Z) + (W * q2.W);
}

// set quaternion to identity
inline core::quaternion& quaternion::makeIdentity() {
    W = 1.f;
    X = 0.f;
    Y = 0.f;
    Z = 0.f;
    return *this;
}

#endif