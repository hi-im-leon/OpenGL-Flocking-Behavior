#include "aRotation.h"
#include <algorithm>
#include <cmath>

#define IS_ZERO(x) ((x)>-EPSILON && (x)<EPSILON)
#define SGN(x) (x>=0?1.0:-1.0)

#define _USE_MATH_DEFINES

#ifndef EPSILON
#define EPSILON 0.001f
#endif

using namespace std;
enum { VX, VY, VZ, VW };

// CONSTRUCTORS
mat3::mat3() 
{
    v[0] = vec3(0.0f,0.0f,0.0f);
    v[1] = v[2] = v[0];
}

mat3::mat3(const vec3& v0, const vec3& v1, const vec3& v2)
{ 
    v[0] = v0; v[1] = v1; v[2] = v2; 
}

mat3::mat3(double d)
{ 
    v[0] = v[1] = v[2] = vec3(d); 
}

mat3::mat3(const mat3& m)
{ 
    v[0] = m.v[0]; v[1] = m.v[1]; v[2] = m.v[2]; 
}

mat3 mat3::Rotation3DDeg(const vec3& axis, double angleDeg)
{
    double angleRad = angleDeg * Deg2Rad;
    return Rotation3DRad(axis, angleRad);
}

mat3 mat3::Rotation3DRad(const vec3& axis, double angleRad)
{
    double c = cos(angleRad), s = sin(angleRad), t = 1.0f - c;
    vec3 Axis = axis;
    Axis.Normalize();
    return mat3(vec3(t * Axis[VX] * Axis[VX] + c,
        t * Axis[VX] * Axis[VY] - s * Axis[VZ],
        t * Axis[VX] * Axis[VZ] + s * Axis[VY]),
        vec3(t * Axis[VX] * Axis[VY] + s * Axis[VZ],
        t * Axis[VY] * Axis[VY] + c,
        t * Axis[VY] * Axis[VZ] - s * Axis[VX]),
        vec3(t * Axis[VX] * Axis[VZ] - s * Axis[VY],
        t * Axis[VY] * Axis[VZ] + s * Axis[VX],
        t * Axis[VZ] * Axis[VZ] + c)
        );
}

mat3 mat3::Rotation3DDeg(const int Axis, double angleDeg)
{
    double  angleRad = angleDeg * Deg2Rad;
    return Rotation3DRad(Axis, angleRad);
}

mat3 mat3::Rotation3DRad(const int Axis, double angleRad)
{
    mat3 m;
    switch(Axis)
    {
    case VX: m = Rotation3DRad(axisX, angleRad);
        break;
    case VY: m = Rotation3DRad(axisY, angleRad);
        break;
    case VZ: m = Rotation3DRad(axisZ, angleRad);
        break;
    }
    return m;
}

// NOTE: You can assume the matrix is orthonormal
bool mat3::ToEulerAnglesXYZ(vec3& angleRad) const
{
    if ((1 - abs(v[0][2])) < EPSILON) {
        angleRad[2] = 0;
        if(v[0][2] > 0) {
            angleRad[1] = M_PI/2;
            angleRad[0] = atan2(v[1][0], v[1][1]);
        } else {
            angleRad[1] = -M_PI/2;
            angleRad[0] = -atan2(v[1][0], v[1][1]);
        }
        return false;
    } else {
        angleRad[1] = asin(v[0][2]);
        angleRad[0] = atan2(-v[1][2]/cos(angleRad[1]), v[2][2]/cos(angleRad[1]));
        angleRad[2] = atan2(-v[0][1]/cos(angleRad[1]), v[0][0]/cos(angleRad[1]));
        return true;
    }
}

bool mat3::ToEulerAnglesXZY(vec3& angleRad) const
{
    if ((1 -abs(v[0][1])) < EPSILON) {
        angleRad[1] = 0;
        if(v[0][1] < 0) {
            angleRad[2] = M_PI/2;
            angleRad[0] = -atan2(-v[2][0], v[2][2]);
        } else {
            angleRad[2] = -M_PI/2;
            angleRad[0] = atan2(-v[2][0], v[2][2]);
        }
        return false;
    }
    else {
        angleRad[2] = asin(-v[0][1]);
        angleRad[0] = atan2(v[2][1]/cos(angleRad[2]), v[1][1]/cos(angleRad[2]));
        angleRad[1] = atan2(v[0][2]/cos(angleRad[2]), v[0][0]/cos(angleRad[2]));
        return true;
    }
}

bool mat3::ToEulerAnglesYXZ(vec3& angleRad) const
{
    if ((1-abs(v[1][2])) < EPSILON) {
        angleRad[2] = 0;
        if(v[1][2] < 0) {
            angleRad[0] = M_PI/2;
            angleRad[1] = -atan2(-v[0][1], v[0][0]);
        } else {
            angleRad[0] = -M_PI/2;
            angleRad[1] = atan2(-v[0][1], v[0][0]);
        }
        return false;
    }
    else {
        angleRad[0] = asin(-v[1][2]);
        angleRad[1] = atan2(v[0][2]/cos(angleRad[0]), v[2][2]/cos(angleRad[0]));
        angleRad[2] = atan2(v[1][0]/cos(angleRad[0]), v[1][1]/cos(angleRad[0]));
        return true;
    }
}

bool mat3::ToEulerAnglesYZX(vec3& angleRad) const
{
    if ((1-abs(v[1][0])) < EPSILON) {
        angleRad[0] = 0;
        if(v[1][0] > 0) {
            angleRad[2] = M_PI/2;
            angleRad[1] = atan2(v[2][1], v[2][2]);
        } else {
            angleRad[2] = -M_PI/2;
            angleRad[1] = -atan2(v[2][1], v[2][2]);
        }
        return false;
    } else {
        angleRad[2] = asin(v[1][0]);
        angleRad[0] = atan2(-v[1][2]/cos(angleRad[2]), v[1][1]/cos(angleRad[2]));
        angleRad[1] = atan2(-v[2][0]/cos(angleRad[2]), v[0][0]/cos(angleRad[2]));
        return true;
    }
}

bool mat3::ToEulerAnglesZXY(vec3& angleRad) const
{
    if ((1-abs(v[2][1])) < EPSILON) {
        angleRad[1] = 0;
        if(v[2][1] > 0) {
            angleRad[0] = M_PI/2;
            angleRad[2] =  atan2(v[0][2], v[0][0]);
        } else {
            angleRad[0] = -M_PI/2;
            angleRad[2] = -atan2(v[0][2], v[0][0]);
        }
        return false;
    } else {
        angleRad[0] = asin(v[2][1]);
        angleRad[1] = atan2(-v[2][0]/cos(angleRad[0]), v[2][2]/cos(angleRad[0]));
        angleRad[2] = atan2(-v[0][1]/cos(angleRad[0]), v[1][1]/cos(angleRad[0]));
        return true;
    }
}

bool mat3::ToEulerAnglesZYX(vec3& angleRad) const
{
    if ((1-abs(v[2][0])) < EPSILON) {
        angleRad[0] = 0;
        if(v[2][0] < 0) {
            angleRad[1] = M_PI/2;
            angleRad[2] = -atan2(-v[1][2], v[1][1]);
        } else {
            angleRad[1] = -M_PI/2;
            angleRad[2] = atan2(-v[1][2], v[1][1]);
        }
        return false;
    } else {
        angleRad[1] = asin(-v[2][0]);
        angleRad[2] = atan2(v[1][0]/cos(angleRad[1]), v[0][0]/cos(angleRad[1]));
        angleRad[0] = atan2(v[2][1]/cos(angleRad[1]), v[2][2]/cos(angleRad[1]));
        return true;
    }
}

mat3 mat3::FromEulerAnglesXYZ(const vec3& angleRad)
{
    mat3 xrotate = Rotation3DRad(axisX, angleRad[0]);
    mat3 yrotate = Rotation3DRad(axisY, angleRad[1]);
    mat3 zrotate = Rotation3DRad(axisZ, angleRad[2]);
    *this = xrotate*yrotate*zrotate;
    return *this;
}

mat3 mat3::FromEulerAnglesXZY(const vec3& angleRad)
{
    mat3 xrotate = Rotation3DRad(axisX, angleRad[0]);
    mat3 yrotate = Rotation3DRad(axisY, angleRad[1]);
    mat3 zrotate = Rotation3DRad(axisZ, angleRad[2]);
    *this = xrotate*zrotate*yrotate;
    return *this;
}

mat3 mat3::FromEulerAnglesYXZ(const vec3& angleRad)
{
    mat3 xrotate = Rotation3DRad(axisX, angleRad[0]);
    mat3 yrotate = Rotation3DRad(axisY, angleRad[1]);
    mat3 zrotate = Rotation3DRad(axisZ, angleRad[2]);
    *this = yrotate*xrotate*zrotate;
    return *this;
}

mat3 mat3::FromEulerAnglesYZX(const vec3& angleRad)
{
    mat3 xrotate = Rotation3DRad(axisX, angleRad[0]);
    mat3 yrotate = Rotation3DRad(axisY, angleRad[1]);
    mat3 zrotate = Rotation3DRad(axisZ, angleRad[2]);
    *this = yrotate*zrotate*xrotate;
    return *this;
}

mat3 mat3::FromEulerAnglesZXY(const vec3& angleRad)
{
    mat3 xrotate = Rotation3DRad(axisX, angleRad[0]);
    mat3 yrotate = Rotation3DRad(axisY, angleRad[1]);
    mat3 zrotate = Rotation3DRad(axisZ, angleRad[2]);
    *this = zrotate*xrotate*yrotate;
    return *this;
}

mat3 mat3::FromEulerAnglesZYX(const vec3& angleRad)
{
    mat3 xrotate = Rotation3DRad(axisX, angleRad[0]);
    mat3 yrotate = Rotation3DRad(axisY, angleRad[1]);
    mat3 zrotate = Rotation3DRad(axisZ, angleRad[2]);
    *this = zrotate*yrotate*xrotate;
    return *this;
}

bool mat3::Reorthogonalize()
{
    // Factor M = QR where Q is orthogonal and R is upper triangular.
    // Algorithm uses Gram-Schmidt orthogonalization (the QR algorithm).
    //
    // If M = [ m0 | m1 | m2 ] and Q = [ q0 | q1 | q2 ], then
    //
    //   q0 = m0/|m0|
    //   q1 = (m1-(q0*m1)q0)/|m1-(q0*m1)q0|
    //   q2 = (m2-(q0*m2)q0-(q1*m2)q1)/|m2-(q0*m2)q0-(q1*m2)q1|
    //
    // where |V| indicates length of vector V and A*B indicates dot
    // product of vectors A and B.  The matrix R has entries
    //
    //   r00 = q0*m0  r01 = q0*m1  r02 = q0*m2
    //   r10 = 0      r11 = q1*m1  r12 = q1*m2
    //   r20 = 0      r21 = 0      r22 = q2*m2
    //
    // The reorthogonalization replaces current matrix by computed Q.

    const double fEpsilon = 1e-05f;

    // unitize column 0
    double fLength = sqrt(v[0][0] * v[0][0] + v[1][0] * v[1][0] + v[2][0] * v[2][0]);
    if ( fLength < fEpsilon )
        return false;
    double fInvLength = 1.0f / fLength;
    v[0][0] *= fInvLength;
    v[1][0] *= fInvLength;
    v[2][0] *= fInvLength;

    // project out column 0 from column 1
    double fDot = v[0][0] * v[0][1] + v[1][0] * v[1][1] + v[2][0] * v[2][1];
    v[0][1] -= fDot * v[0][0];
    v[1][1] -= fDot * v[1][0];
    v[2][1] -= fDot * v[2][0];

    // unitize column 1
    fLength = sqrt(v[0][1] * v[0][1] + v[1][1] * v[1][1] + v[2][1] * v[2][1]);
    if ( fLength < fEpsilon )
        return false;
    fInvLength = 1.0f/fLength;
    v[0][1] *= fInvLength;
    v[1][1] *= fInvLength;
    v[2][1] *= fInvLength;

    // project out column 0 from column 2
    fDot = v[0][0] * v[0][2] + v[1][0] * v[1][2] + v[2][0] * v[2][2];
    v[0][2] -= fDot * v[0][0];
    v[1][2] -= fDot * v[1][0];
    v[2][2] -= fDot * v[2][0];

    // project out column 1 from column 2
    fDot = v[0][1] * v[0][2] + v[1][1] * v[1][2] + v[2][1] * v[2][2];
    v[0][2] -= fDot * v[0][1];
    v[1][2] -= fDot * v[1][1];
    v[2][2] -= fDot * v[2][1];

    // unitize column 2
    fLength = sqrt(v[0][2] * v[0][2] + v[1][2] * v[1][2] + v[2][2] * v[2][2]);
    if ( fLength < fEpsilon )
        return false;
    fInvLength = 1.0f / fLength;
    v[0][2] *= fInvLength;
    v[1][2] *= fInvLength;
    v[2][2] *= fInvLength;

    return true;
}

// Conversion with Quaternion
Quaternion mat3::ToQuaternion() const
{
    Quaternion q;
    q.FromRotation(*this);
    return q;
}

void mat3::FromQuaternion(const Quaternion& q)
{
    (*this) = q.ToRotation();
}


void mat3::ToAxisAngle(vec3& axis, double& angleRad) const
{
    // Let (x,y,z) be the unit-length axis and let A be an angle of rotation.
    // The rotation matrix is R = I + sin(A)*P + (1-cos(A))*P^2 where
    // I is the identity and
    //
    //       +-        -+
    //   P = |  0 +z -y |
    //       | -z  0 +x |
    //       | +y -x  0 |
    //       +-        -+
    //
    // Some algebra will show that
    //
    //   cos(A) = (trace(R)-1)/2  and  R - R^t = 2*sin(A)*P

    double fTrace = v[0][0] + v[1][1] + v[2][2];
    angleRad = acos( 0.5f * (fTrace - 1.0f));

    axis[VX] = v[1][2] - v[2][1];
    axis[VY] = v[2][0] - v[0][2];
    axis[VZ] = v[0][1] - v[1][0];
    double fLength = axis.Length();
    const double fEpsilon = 1e-06f;
    if ( fLength > fEpsilon )
    {
        double fInvLength = 1.0f / fLength;
        axis *= -fInvLength;
    }
    else  // angle is 0 or pi
    {
        if ( angleRad > 1.0f )  // any number strictly between 0 and pi works
        {
            // angle must be pi
            axis[VX] = sqrt(0.5f * (1.0f + v[0][0]));
            axis[VY] = sqrt(0.5f * (1.0f + v[1][1]));
            axis[VZ] = sqrt(0.5f * (1.0f + v[2][2]));

            // determine signs of axis components
            double tx, ty, tz;
            tx = v[0][0] * axis[VX] + v[0][1] * axis[VY] + v[0][2] * axis[VZ] - axis[VX];
            ty = v[1][0] * axis[VX] + v[1][1] * axis[VY] + v[1][2] * axis[VZ] - axis[VY];
            tz = v[2][0] * axis[VX] + v[2][1] * axis[VY] + v[2][2] * axis[VZ] - axis[VZ];
            fLength = tx * tx + ty * ty + tz * tz;
            if ( fLength < fEpsilon )
            {
               axis = -axis;
               return;
            }

            axis[VZ] = -axis[VZ];
            tx = v[0][0] * axis[VX] + v[0][1] * axis[VY] + v[0][2] * axis[VZ] - axis[VX];
            ty = v[1][0] * axis[VX] + v[1][1] * axis[VY] + v[1][2] * axis[VZ] - axis[VY];
            tz = v[2][0] * axis[VX] + v[2][1] * axis[VY] + v[2][2] * axis[VZ] - axis[VZ];
            fLength = tx * tx + ty * ty + tz * tz;
            if ( fLength < fEpsilon )
            {
               axis = -axis;
               return;
            }

            axis[VY] = -axis[VY];
            tx = v[0][0] * axis[VX] + v[0][1] * axis[VY] + v[0][2] * axis[VZ] - axis[VX];
            ty = v[1][0] * axis[VX] + v[1][1] * axis[VY] + v[1][2] * axis[VZ] - axis[VY];
            tz = v[2][0] * axis[VX] + v[2][1] * axis[VY] + v[2][2] * axis[VZ] - axis[VZ];
            fLength = tx * tx + ty * ty + tz * tz;
            if ( fLength < fEpsilon )
            {
               axis = -axis;
               return;
            }
        }
        else
        {
            // Angle is zero, matrix is the identity, no unique axis, so
            // return (0,1,0) for as good a guess as any.
            angleRad = 0.0f;
            axis[VX] = 0.0f;
            axis[VY] = 1.0f;
            axis[VZ] = 0.0f;
        }
    }
}

mat3 mat3::FromToRotation(const vec3& fromDir, const vec3& toDir)
{
    vec3 dir1 = fromDir;
    vec3 dir2 = toDir;
    dir1.Normalize();
    dir2.Normalize();
    vec3 axis = dir1.Cross(dir2);
    double v = std::min(1.0, std::max(-1.0, dir1 * dir2));
    double angle = acos(v);
    mat3 mat;
    mat.FromAxisAngle(axis, angle);
    //vec3 tmp1 = mat * dir1;
    //std::cout << "CHECK " << tmp1 << dir2 << std::endl; 
    return mat;
}

mat3 mat3::Inverse() const    // Gauss-Jordan elimination with partial pivoting
{
    mat3 a(*this),        // As a evolves from original mat into identity
    b(identity3D);   // b evolves from identity into inverse(a)
    int     i, j, i1;

    // Loop over cols of a from left to right, eliminating above and below diag
    for (j=0; j<3; j++) {   // Find largest pivot in column j among rows j..2
        i1 = j;    	    // Row with largest pivot candidate
        for (i=j+1; i<3; i++)
            if (abs(a.v[i].n[j]) > abs(a.v[i1].n[j]))
            	i1 = i;

        // Swap rows i1 and j in a and b to put pivot on diagonal
        Swap(a.v[i1], a.v[j]);
        Swap(b.v[i1], b.v[j]);

        // Scale row j to have a unit diagonal
        if (a.v[j].n[j] == 0.)
        {
            std::cout << "mat3::inverse: singular matrix; can't invert\n";
            return b;
        }
        b.v[j] /= a.v[j].n[j];
        a.v[j] /= a.v[j].n[j];

        // Eliminate off-diagonal elements in col j of a, doing identical ops to b
        for (i=0; i<3; i++)
            if (i!=j) 
            {
            	b.v[i] -= a.v[i].n[j]*b.v[j];
            	a.v[i] -= a.v[i].n[j]*a.v[j];
            }
    }
    return b;
}

void mat3::FromAxisAngle(const vec3& axis, double angleRad)
{
    *this = Rotation3DRad(axis, angleRad);
}


// ASSIGNMENT OPERATORS

mat3& mat3::operator = ( const mat3& m )
{ 
    v[0] = m.v[0]; v[1] = m.v[1]; v[2] = m.v[2]; 
    return *this; 
}

mat3& mat3::operator += ( const mat3& m )
{ 
    v[0] += m.v[0]; v[1] += m.v[1]; v[2] += m.v[2]; 
    return *this; 
}

mat3& mat3::operator -= ( const mat3& m )
{ 
    v[0] -= m.v[0]; v[1] -= m.v[1]; v[2] -= m.v[2]; 
    return *this; 
}

mat3& mat3::operator *= ( double d )
{ 
    v[0] *= d; v[1] *= d; v[2] *= d; 
    return *this; 
}

mat3& mat3::operator /= ( double d )
{ 
    v[0] /= d; v[1] /= d; v[2] /= d; 
    return *this; 
}

vec3& mat3::operator [] ( int i) 
{
    assert(! (i < VX || i > VZ));
    return v[i];
}

const vec3& mat3::operator [] ( int i) const 
{
    assert(!(i < VX || i > VZ));
    return v[i];
}

// SPECIAL FUNCTIONS

mat3 mat3::Transpose() const 
{
    return mat3(vec3(v[0][0], v[1][0], v[2][0]),
        vec3(v[0][1], v[1][1], v[2][1]),
        vec3(v[0][2], v[1][2], v[2][2]));
}


void mat3::WriteToGLMatrix(float* m) const
{
    m[0] = v[0][0]; m[4] = v[0][1]; m[8] = v[0][2];  m[12] = 0.0f;
    m[1] = v[1][0]; m[5] = v[1][1]; m[9] = v[1][2];  m[13] = 0.0f;
    m[2] = v[2][0]; m[6] = v[2][1]; m[10] = v[2][2]; m[14] = 0.0f;
    m[3] = 0.0f;    m[7] = 0.0f;    m[11] = 0.0f;    m[15] = 1.0f;
}

void mat3::ReadFromGLMatrix(float* m)
{
    v[0][0] = m[0]; v[0][1] = m[4]; v[0][2] = m[8];
    v[1][0] = m[1]; v[1][1] = m[5]; v[1][2] = m[9];
    v[2][0] = m[2]; v[2][1] = m[6]; v[2][2] = m[10];
}

vec3 mat3::GetRow(unsigned int axis) const
{
    vec3 rowVec = v[axis];
    return rowVec;
}

vec3 mat3::GetCol(unsigned int axis) const
{
    vec3 colVec;
    colVec[0] = v[0][axis]; colVec[1] = v[1][axis]; colVec[2] = v[2][axis];
    return colVec;
}

void mat3::SetRow(unsigned int axis, const vec3& rowVec)
{
    v[axis] = rowVec;
}

void mat3::SetCol(unsigned int axis, const vec3& colVec)
{
    v[0][axis] = colVec[0]; v[1][axis] = colVec[1]; v[2][axis] = colVec[2];
}

vec3 mat3::GetYawPitchRoll(unsigned int leftAxis, unsigned int upAxis, unsigned int frontAxis) const
{
    // Assume world coordinates: Y up, X left, Z front.

    vec3 leftVect, upVect, frontVect, dVect, angles, frontVect2, leftVect2;
    double t, value, x, y;
    leftVect = GetCol(leftAxis);
    upVect = GetCol(upAxis);
    frontVect = GetCol(frontAxis);

    // Compute yaw angle
    if (frontVect[VY] >= 0.0f && upVect[VY] >= 0.0f)
    {
        frontVect2 = frontVect;
        dVect = -upVect - frontVect2;
    }else if (frontVect[VY] < 0.0f && upVect[VY] < 0.0f)
    {
        frontVect2 = -frontVect;
        dVect = upVect - frontVect2;
    }else if (frontVect[VY] >= 0.0f && upVect[VY] < 0.0f)
    {
        frontVect2 = -frontVect;
        dVect = -upVect - frontVect2;

    }else if (frontVect[VY] < 0.0f && upVect[VY] >= 0.0f)
    {
        frontVect2 = frontVect;
        dVect = upVect - frontVect2;
    }
    t = -frontVect2[VY] / dVect[VY];
    x = frontVect2[VZ] + t * dVect[VZ];
    y = frontVect2[VX] + t * dVect[VX];
    angles[0] = atan2(y, x);
    frontVect2 = vec3(y, 0.0f, x);
    frontVect2.Normalize();
    leftVect2 = vec3(0.0f, 1.0f, 0.0f);
    leftVect2 = leftVect2.Cross(frontVect2);

    // Compute pitch angle
    double v = acos(frontVect * frontVect2);
    if (frontVect[VY] >= 0.0f)
    {
        value = -v;
    }else
    {
        value = v;
    }
    angles[1] = value;

    // Compute roll angle
    v = acos(leftVect * leftVect2);
    if (leftVect[VY] >= 0.0f)
    {
        value = -v;
    }else
    {
        value = v;
    }
    angles[2] = value;

    return angles;
}

//OpenGL transformation matrix
//    Rx =
//    |1       0        0    Tx|
//    |0  cos(a)  -sin(a)    Ty|
//    |0  sin(a)   cos(a)    Tz|
//    |0       0        0    1 |
//
//    Ry =
//    | cos(a)  0  sin(a)    Tx|
//    |      0  1       0    Ty|
//    |-sin(a)  0  cos(a)    Tz|
//    |      0  0       0    1 |
//
//    Rz = 
//    |cos(a)  -sin(a)  0   Tx|
//    |sin(a)   cos(a)  0   Ty|
//    |     0        0  1   Tz|
//    |     0        0  0   1 |
//
// However, when they are stored in OpenGL matrix, they are stored column major
// OpenGL convention
// m[0] = R[0][0]; m[4] = R[0][1]; m[8]  = R[0][2]; m[12] = Tx;
// m[1] = R[1][0]; m[5] = R[1][1]; m[9]  = R[1][2]; m[13] = Ty;
// m[2] = R[2][0]; m[6] = R[2][1]; m[10] = R[2][2]; m[14] = Tz;
// m[3] = 0.0f;    m[7] = 0.0f;    m[11] = 0.0f;    m[15] = 1.0f;

// FRIENDS

mat3 operator - (const mat3& a)
{ 
    return mat3(-a.v[0], -a.v[1], -a.v[2]); 
}

mat3 operator + (const mat3& a, const mat3& b)
{ 
    return mat3(a.v[0] + b.v[0], a.v[1] + b.v[1], a.v[2] + b.v[2]); 
}

mat3 operator - (const mat3& a, const mat3& b)
{ 
    return mat3(a.v[0] - b.v[0], a.v[1] - b.v[1], a.v[2] - b.v[2]); 
}

mat3 operator * (const mat3& a, const mat3& b)
{
#define ROWCOL(i, j) \
    a.v[i].n[0]*b.v[0][j] + a.v[i].n[1]*b.v[1][j] + a.v[i].n[2]*b.v[2][j]
    return mat3(vec3(ROWCOL(0,0), ROWCOL(0,1), ROWCOL(0,2)),
        vec3(ROWCOL(1,0), ROWCOL(1,1), ROWCOL(1,2)),
        vec3(ROWCOL(2,0), ROWCOL(2,1), ROWCOL(2,2)));
#undef ROWCOL // (i, j)
}

mat3 operator * (const mat3& a, double d)
{ 
    return mat3(a.v[0] * d, a.v[1] * d, a.v[2] * d); 
}

mat3 operator * (double d, const mat3& a)
{ 
    return a*d; 
}

mat3 operator / (const mat3& a, double d)
{ 
    return mat3(a.v[0] / d, a.v[1] / d, a.v[2] / d); 
}

int operator == (const mat3& a, const mat3& b)
{ 
    return (a.v[0] == b.v[0]) && (a.v[1] == b.v[1]) && (a.v[2] == b.v[2]); 
}

int operator != (const mat3& a, const mat3& b)
{ 
    return !(a == b); 
}

void Swap(mat3& a, mat3& b)
{ 
    mat3 tmp(a); a = b; b = tmp; 
}

std::istream& operator >> (std::istream& s, mat3& v)
{
    double value;
    for (unsigned int i = 0; i < 3; i++)
        for (unsigned int j = 0; j < 3; j++)
        {
            s >> value;
            v[i][j] = value;
        }
    return s;
}

std::ostream& operator << (std::ostream& s, const mat3& v)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        for (unsigned int j = 0; j < 2; j++)
        {
            s << (float) v[i][j] << " ";
        }
        s << (float) v[i][2] << std::endl;
    }
    return s;
}


 vec3 operator * (const mat3& a, const vec3& v)
{
#define ROWCOL(i) a.v[i].n[0]*v.n[VX] + a.v[i].n[1]*v.n[VY] \
    + a.v[i].n[2]*v.n[VZ]
    return vec3(ROWCOL(0), ROWCOL(1), ROWCOL(2));
#undef ROWCOL // (i)
}


Quaternion::Quaternion()
{
    n[VW] = 0; n[VX] = 0; n[VY] = 0; n[VZ] = 0;
}

Quaternion::Quaternion(double w, double x, double y, double z)
{
    n[VW] = w; n[VX] = x; n[VY] = y; n[VZ] = z;
}

Quaternion::Quaternion(const Quaternion& q)
{
    n[VW] = q.n[VW]; n[VX] = q.n[VX]; n[VY] = q.n[VY]; n[VZ] = q.n[VZ];
}

// Static functions

double Quaternion::Distance(const Quaternion& q1, const Quaternion& q2) // returns angle between in radians
{
    double inner_product = Quaternion::Dot(q1, q2);
    if (inner_product < 0.0)
    {
        inner_product = Quaternion::Dot(q1, -q2);
    }

    double tmp = std::min<double>(1.0, std::max<double>(-1.0,2*inner_product*inner_product - 1));
    double theta = acos(tmp);
    return theta;
}

double Quaternion::Dot(const Quaternion& q0, const Quaternion& q1)
{
    return q0.n[VW] * q1.n[VW] + q0.n[VX] * q1.n[VX] + q0.n[VY] * q1.n[VY] + q0.n[VZ] * q1.n[VZ];
}

Quaternion Quaternion::UnitInverse(const Quaternion& q)
{
    return Quaternion(q.n[VW], -q.n[VX], -q.n[VY], -q.n[VZ]);
}

// Assignment operators
Quaternion& Quaternion::operator = (const Quaternion& q)
{
    n[VW] = q.n[VW]; n[VX] = q.n[VX]; n[VY] = q.n[VY]; n[VZ] = q.n[VZ];
    return *this;
}

Quaternion& Quaternion::operator += (const Quaternion& q)
{
    n[VW] += q.n[VW]; n[VX] += q.n[VX]; n[VY] += q.n[VY]; n[VZ] += q.n[VZ];
    return *this;
}

Quaternion& Quaternion::operator -= (const Quaternion& q)
{
    n[VW] -= q.n[VW]; n[VX] -= q.n[VX]; n[VY] -= q.n[VY]; n[VZ] -= q.n[VZ];
    return *this;
}

Quaternion& Quaternion::operator *= (const Quaternion& q)
{
    *this = Quaternion(n[VW] * q.n[VW] - n[VX] * q.n[VX] - n[VY] * q.n[VY] - n[VZ] * q.n[VZ],
        n[VW] * q.n[VX] + n[VX] * q.n[VW] + n[VY] * q.n[VZ] - n[VZ] * q.n[VY],
        n[VW] * q.n[VY] + n[VY] * q.n[VW] + n[VZ] * q.n[VX] - n[VX] * q.n[VZ],
        n[VW] * q.n[VZ] + n[VZ] * q.n[VW] + n[VX] * q.n[VY] - n[VY] * q.n[VX]);
    return *this;
}

Quaternion& Quaternion::operator *= (double d)
{
    n[VW] *= d; n[VX] *= d;    n[VY] *= d; n[VZ] *= d;
    return *this;
}

Quaternion& Quaternion::operator /= (double d)
{
    n[VW] /= d; n[VX] /= d;    n[VY] /= d; n[VZ] /= d;
    return *this;
}

// Indexing
double& Quaternion::operator [](int i)
{
    return n[i];
}

double Quaternion::operator [](int i) const
{
    return n[i];
}

double& Quaternion::W()
{
    return n[VW];
}

double Quaternion::W() const
{
    return n[VW];
}

double& Quaternion::X()
{
    return n[VX];
}

double Quaternion::X() const
{
    return n[VX];
}

double& Quaternion::Y()
{
    return n[VY];
}

double Quaternion::Y() const
{
    return n[VY];
}

double& Quaternion::Z()
{
    return n[VZ];
}

double Quaternion::Z() const
{
    return n[VZ];
}

// Friends

Quaternion operator - (const Quaternion& q)
{
    return Quaternion(-q.n[VW], -q.n[VX], -q.n[VY], -q.n[VZ]); 
}

Quaternion operator + (const Quaternion& q0, const Quaternion& q1)
{
    return Quaternion(q0.n[VW] + q1.n[VW], q0.n[VX] + q1.n[VX], q0.n[VY] + q1.n[VY], q0.n[VZ] + q1.n[VZ]);
}

Quaternion operator - (const Quaternion& q0, const Quaternion& q1)
{
    return Quaternion(q0.n[VW] - q1.n[VW], q0.n[VX] - q1.n[VX], q0.n[VY] - q1.n[VY], q0.n[VZ] - q1.n[VZ]);
}

Quaternion operator * (const Quaternion& q, double d)
{
    return Quaternion(q.n[VW] * d, q.n[VX] * d, q.n[VY] * d, q.n[VZ] * d);
}

Quaternion operator * (double d, const Quaternion& q)
{
    return Quaternion(q.n[VW] * d, q.n[VX] * d, q.n[VY] * d, q.n[VZ] * d);
}

Quaternion operator * (const Quaternion& q0, const Quaternion& q1)
{
    return Quaternion(q0.n[VW] * q1.n[VW] - q0.n[VX] * q1.n[VX] - q0.n[VY] * q1.n[VY] - q0.n[VZ] * q1.n[VZ],
        q0.n[VW] * q1.n[VX] + q0.n[VX] * q1.n[VW] + q0.n[VY] * q1.n[VZ] - q0.n[VZ] * q1.n[VY],
        q0.n[VW] * q1.n[VY] + q0.n[VY] * q1.n[VW] + q0.n[VZ] * q1.n[VX] - q0.n[VX] * q1.n[VZ],
        q0.n[VW] * q1.n[VZ] + q0.n[VZ] * q1.n[VW] + q0.n[VX] * q1.n[VY] - q0.n[VY] * q1.n[VX]);
}

Quaternion operator / (const Quaternion& q, double d)
{
    return Quaternion(q.n[VW] / d, q.n[VX] / d, q.n[VY] / d, q.n[VZ] / d);
}

bool operator == (const Quaternion& q0, const Quaternion& q1)
{
    return (q0.n[VW] == q1.n[VW]) && (q0.n[VX] == q1.n[VX]) && (q0.n[VY] == q1.n[VY]) && (q0.n[VZ] == q1.n[VZ]);
}

bool operator != (const Quaternion& q0, const Quaternion& q1)
{
    return !(q0 == q1); 
}

// special functions

double Quaternion::SqrLength() const
{
    return n[VW] * n[VW] + n[VX] * n[VX] + n[VY] * n[VY] + n[VZ] * n[VZ];
}

double Quaternion::Length() const
{
    double l = SqrLength();
    if (l > EPSILON)
        return sqrt(SqrLength());
    else 
        return 0;
}

Quaternion& Quaternion::Normalize()
{
    double l = Length();
    if (l < EPSILON || abs(l) > 1e6)
    {
        FromAxisAngle(vec3(0.0f, 1.0f, 0.0f), 0.0f);
    }else
    {
        *this /= l;
    }

    return *this; 
}


Quaternion Quaternion::Conjugate() const
{
    return Quaternion(n[VW], -n[VX], -n[VY], -n[VZ]);
}

Quaternion Quaternion::Inverse() const
{
    return Conjugate() / SqrLength();
}

Quaternion Quaternion::Exp(const Quaternion& q)
{
    // q = A*(x*i+y*j+z*k) where (x,y,z) is unit length
    // exp(q) = cos(A)+sin(A)*(x*i+y*j+z*k)
    double angle = sqrt(q.n[VX] * q.n[VX] + q.n[VY] * q.n[VY] + q.n[VZ] * q.n[VZ]);
    double sn, cs;
    sn = sin(angle);
    cs = cos(angle);

    // When A is near zero, sin(A)/A is approximately 1.  Use
    // exp(q) = cos(A)+A*(x*i+y*j+z*k)
    double coeff = ( abs(sn) < EPSILON ? 1.0f : sn/angle );

    Quaternion result(cs, coeff * q.n[VX], coeff * q.n[VY], coeff * q.n[VZ]);

    return result;
}

Quaternion Quaternion::Log(const Quaternion& q)
{
    // q = cos(A)+sin(A)*(x*i+y*j+z*k) where (x,y,z) is unit length
    // log(q) = A*(x*i+y*j+z*k)
    
    double angle = acos(q.n[VW]);
    double sn = sin(angle);

    // When A is near zero, A/sin(A) is approximately 1.  Use
    // log(q) = sin(A)*(x*i+y*j+z*k)
    double coeff = ( abs(sn) < EPSILON ? 1.0f : angle/sn );

    return Quaternion(0.0f, coeff * q.n[VX], coeff * q.n[VY], coeff * q.n[VZ]);
}

void Quaternion::Zero()
{
    n[VW] = n[VX] = n[VY] = n[VZ] = 0.0f;
}

Quaternion Quaternion::Slerp(const Quaternion& q0, const Quaternion& q1, double t)
{
    double angle = Quaternion::Dot(q0, q1);
    return (sin((1-t)*(angle))/sin(angle))*q0 + (sin(t*angle)/sin(angle))*q1;
}

Quaternion Quaternion::Intermediate (const Quaternion& q0, const Quaternion& _q1, const Quaternion& _q2)
{
    // Return the intermediate quaternion based on the inputs  
    return Quaternion();
}

Quaternion Quaternion::Squad(const Quaternion& q0, const Quaternion& a, const Quaternion& b, const Quaternion& q1, double t)
{
    // Return the result of Squad based on the input quaternions
    return Quaternion();
}

vec3 Quaternion::ToExpMap() const
{
    vec3 axis; double angle;
    ToAxisAngle(axis, angle);

    vec3 expmap(0,0,0);
    if (abs(angle) > 0.000001)
    {
        double factor = angle / sin(0.5*angle);
        expmap = vec3(factor * X(), factor * Y(), factor * Z());
    }
    return expmap;
}

void Quaternion::FromExpMap(const vec3& expmap)
{
    double theta = expmap.Length();
    double scale = 0.0;
    if (abs(theta) < 0.032)
    {
        scale = 0.5 + (theta*theta)*0.021;
    }
    else
    {
        scale = sin(0.5*theta)/theta;
    }
    n[VW] = cos(0.5*theta);
    n[VX] = expmap[0]*scale;
    n[VY] = expmap[1]*scale;
    n[VZ] = expmap[2]*scale;
    Normalize(); // not sure this is necessary....
}

Quaternion Quaternion::ProjectToAxis(const Quaternion& q, vec3& axis)
{
    axis.Normalize();
    vec3 qv = vec3(q.X(), q.Y(), q.Z());
    double angle = acos(q.W());
    double sn = sin(angle);
    vec3 qaxis = qv / sn;
    qaxis.Normalize();
    angle = qaxis * axis;
    double halfTheta;
    if (angle < EPSILON)
    {
        halfTheta = 0.0f;
    }else
    {
        double s = axis * qv;
        double c = q.W();
        halfTheta = atan2(s, c);
    }    
    double cn = cos(halfTheta);
    sn = sin(halfTheta);
    return Quaternion(cn, sn * axis[VX], sn * axis[VY], sn * axis[VZ]); 
}

// Conversion functions
void Quaternion::ToAxisAngle (vec3& axis, double& angleRad) const
{
    angleRad = acos(this->W())*2;
    double sin_val = sin(angleRad/2);
    if (abs(sin_val) > EPSILON) { 
        axis = vec3(this->X()/sin_val, this->Y()/sin_val, this->Z()/sin_val);
    } else {
        axis = vec3(this->X(), this->Y(), this->Z());
    }
}

void Quaternion::FromAxisAngle (const vec3& axis, double angleRad)
{
    this->W() = cos(angleRad/2);
    vec3 normalized = axis;
    normalized = normalized.Normalize();
    double sin_val = sin(angleRad/2);
    if (abs(sin_val) > EPSILON) {
        this->X() = normalized[0]*sin_val;
        this->Y() = normalized[1]*sin_val;
        this->Z() = normalized[2]*sin_val;
    } else {
        this->X() = normalized[0];
        this->Y() = normalized[1];
        this->Z() = normalized[2];
    }
}

mat3 Quaternion::ToRotation () const
{
    double w = this->W();
    double x = this->X();
    double y = this->Y();
    double z = this->Z();
    double length = sqrt(w*w+x*x+y*y+z*z);
    w = w/length;
    x = x/length;
    y = y/length;
    z = z/length;
    return mat3(
        vec3(1-2*y*y-2*z*z, 2*x*y+2*w*z, 2*x*z-2*w*y),
        vec3(2*x*y-2*w*z, 1-2*x*x-2*z*z, 2*y*z+2*w*x),
        vec3(2*x*z+2*w*y, 2*y*z-2*w*x, 1-2*x*x-2*y*y)
        ).Transpose();
}

void Quaternion::FromRotation(const mat3& rot)
{
    double w, x, y, z;
    w = (1.0/4.0)*(1+rot[0][0]+rot[1][1]+rot[2][2]);
    if (w > EPSILON) {
        w = sqrt(w);
        x = (-1)*(rot[1][2]-rot[2][1])/(4*w);
        y = (-1)*(rot[2][0]-rot[0][2])/(4*w);
        z = (-1)*(rot[0][1]-rot[1][0])/(4*w);
    } else {
        w = 0;
        x = (-1.0/2.0)*(rot[1][1] + rot[2][2]);
        if (x > EPSILON) {
            x = sqrt(x);
            y = (rot[0][1])/(2*x);
            z = (rot[0][2])/(2*x);
        } else {
            x = 0;
            y = (1.0/2.0)*(1-rot[2][2]);
            if(y > EPSILON) {
                y = sqrt(y);
                z = (rot[1][2])/(2*y);
            } else {
                y = 0;
                z = 1;
            }
        }
    }
    this->W() = w;
    this->X() = x;
    this->Y() = y;
    this->Z() = z;
}

std::istream& operator >> (std::istream& s, Quaternion& v)
{
    double x, y, z, w;
    s >> w >> x >> y >> z;
    v[VX] = x;
    v[VY] = y;
    v[VZ] = z;
    v[VW] = w;
    return s;
}

std::ostream& operator << (std::ostream& s, const Quaternion& v)
{
    s << (float) v[VW] << " " << (float) v[VX] << " " << (float) v[VY] << " " << (float) v[VZ];
    return s;
}
