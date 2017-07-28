#include "aTransform.h"

Transform::Transform() : 
    m_rotation(identity3D),
    m_translation(vec3Zero)
{
}

Transform::Transform(const mat3& rot, const vec3& offset) : 
    m_rotation(rot), m_translation(offset)
{
}

Transform::Transform(const Transform& m)
{
    *this = m;
}

// Assignment operators
Transform& Transform::operator = (const Transform& orig)
{
    if (&orig == this)
    {
        return *this;
    }
    m_rotation = orig.m_rotation;
    m_translation = orig.m_translation;
    return *this;
}

Transform Transform::Inverse() const
{
    mat3 transpose = m_rotation.Transpose();
    vec3 inverse_vector = (-1)*transpose*m_translation;
    return Transform(transpose, inverse_vector);
}

void Transform::WriteToGLMatrix(float* m)
{
    m[0] = m_rotation[0][0]; m[4] = m_rotation[0][1]; m[8] = m_rotation[0][2];  m[12] = m_translation[0];
    m[1] = m_rotation[1][0]; m[5] = m_rotation[1][1]; m[9] = m_rotation[1][2];  m[13] = m_translation[1];
    m[2] = m_rotation[2][0]; m[6] = m_rotation[2][1]; m[10] = m_rotation[2][2]; m[14] = m_translation[2];
    m[3] = 0.0f;    m[7] = 0.0f;    m[11] = 0.0f;    m[15] = 1.0f;
}

void Transform::ReadFromGLMatrix(float* m)
{
    m_rotation[0][0] = m[0]; m_rotation[0][1] = m[4]; m_rotation[0][2] = m[8];  m_translation[0] = m[12];
    m_rotation[1][0] = m[1]; m_rotation[1][1] = m[5]; m_rotation[1][2] = m[9];  m_translation[1] = m[13];
    m_rotation[2][0] = m[2]; m_rotation[2][1] = m[6]; m_rotation[2][2] = m[10]; m_translation[2] = m[14];
}

vec3 Transform::TransformPoint(const vec3& pos) const
{
    return m_rotation*pos + m_translation;
}

vec3 Transform::TransformVector(const vec3& dir) const
{
    return m_rotation*dir;
}

Transform operator * (const Transform& t1, const Transform& t2)
{
    mat3 a1 = t1.m_rotation;
    mat3 a2 = t2.m_rotation;
    vec3 b1 = t1.m_translation;
    vec3 b2 = t2.m_translation;
    Transform tmp(a1*a2, a1*b2 + b1);
    return tmp;
}

std::ostream& operator << (std::ostream& s, const Transform& t)
{
    vec3 anglesRad;
    t.m_rotation.ToEulerAnglesZXY(anglesRad);
    s << "R: " << anglesRad << " T: " << t.m_translation << " ";
    return s;
}

vec3 operator * (const Transform& a, const vec3& v)
{
    return a.TransformPoint(v);
}



