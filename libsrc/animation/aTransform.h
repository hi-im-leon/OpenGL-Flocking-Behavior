#ifndef aTransform_H_
#define aTransform_H_

#include "aRotation.h"
#include "aVector.h"

class Transform
{
public:
    Transform();
    Transform(const mat3& rotation, const vec3& translation);
    Transform(const Transform& transform);
    Transform& operator = (const Transform& source); 

    Transform Inverse() const;
    void WriteToGLMatrix(float* m);							// turn rotational data into 4x4 opengl matrix 
    void ReadFromGLMatrix(float* m);						// read rotational data from 4x4 opengl matrix
    vec3 TransformPoint(const vec3& pos) const;
    vec3 TransformVector(const vec3& dir) const;

    friend Transform operator * (const Transform& a, const Transform& b);		// m1 * m2
    friend vec3 operator * (const Transform& a, const vec3& v);	    // linear transform, assumes a point
    friend std::ostream& operator << (std::ostream& s, const Transform& v);

public:
    mat3 m_rotation;
    vec3 m_translation;
};
#endif

