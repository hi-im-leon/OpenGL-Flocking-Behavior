#ifndef aRotation_H_
#define aRotation_H_

#include <iostream>
#include "aVector.h"

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif
#ifndef M_PI_2
#define M_PI_2 1.5707963267948966192313216916398
#endif

const double Rad2Deg = (180.0f / M_PI);			// Rad to Degree
const double Deg2Rad = (M_PI / 180.0f);			// Degree to Rad

class Quaternion;
class mat3
{
protected:

    vec3 v[3];

public:

    // Constructors
    mat3();
    mat3(const vec3& v0, const vec3& v1, const vec3& v2);
    mat3(double d);
    mat3(const mat3& m);

    // Static functions
    //static mat3 Identity();
    static mat3 Rotation3DDeg(const vec3& axis, double angleDeg);
    static mat3 Rotation3DRad(const vec3& axis, double angleRad);
    static mat3 Rotation3DDeg(const int Axis, double angleDeg);
    static mat3 Rotation3DRad(const int Axis, double angleRad);
    static mat3 FromToRotation(const vec3& fromDir, const vec3& toDir);

    // Rotation operations, matrix must be orthonomal
    bool ToEulerAnglesXYZ(vec3& anglesRad) const;
    bool ToEulerAnglesXZY(vec3& anglesRad) const;
    bool ToEulerAnglesYXZ(vec3& anglesRad) const;
    bool ToEulerAnglesYZX(vec3& anglesRad) const;
    bool ToEulerAnglesZXY(vec3& anglesRad) const;
    bool ToEulerAnglesZYX(vec3& anglesRad) const;
    mat3 FromEulerAnglesXYZ(const vec3& anglesRad);
    mat3 FromEulerAnglesXZY(const vec3& anglesRad);
    mat3 FromEulerAnglesYXZ(const vec3& anglesRad);
    mat3 FromEulerAnglesYZX(const vec3& anglesRad);
    mat3 FromEulerAnglesZXY(const vec3& anglesRad);
    mat3 FromEulerAnglesZYX(const vec3& anglesRad);

    // Conversion with Quaternion
    Quaternion ToQuaternion() const;
    void FromQuaternion(const Quaternion& q);
    void ToAxisAngle(vec3& axis, double& angleRad) const;
    void FromAxisAngle(const vec3& axis, double angleRad);

    // Assignment operators
    mat3& operator = ( const mat3& m );	    // assignment of a mat3
    mat3& operator += ( const mat3& m );	    // incrementation by a mat3
    mat3& operator -= ( const mat3& m );	    // decrementation by a mat3
    mat3& operator *= ( double d );	    // multiplication by a constant
    mat3& operator /= ( double d );	    // division by a constant
    vec3& operator [] ( int i);					// indexing
    const vec3& operator [] ( int i) const;		// read-only indexing

    // special functions
    mat3 Transpose() const;								// transpose
    mat3 Inverse() const;								// inverse
    void WriteToGLMatrix(float* m) const;							// turn rotational data into 4x4 opengl matrix with zero translation
    void ReadFromGLMatrix(float* m);						// read rotational data from 4x4 opengl matrix
    bool Reorthogonalize();								// Gram-Schmidt orthogonalization
    vec3 GetRow(unsigned int axis) const;	// get a particular row
    vec3 GetCol(unsigned int axis) const;	// get a particular col
    void SetRow(unsigned int axis, const vec3& rowVec);	// set a particular row
    void SetCol(unsigned int axis, const vec3& colVec);	// set a particular col
    vec3 GetYawPitchRoll(unsigned int leftAxis, unsigned int upAixs, unsigned int frontAxis) const;

    // friends
     friend mat3 operator - (const mat3& a);						// -m1
     friend mat3 operator + (const mat3& a, const mat3& b);	    // m1 + m2
     friend mat3 operator - (const mat3& a, const mat3& b);	    // m1 - m2
     friend mat3 operator * (const mat3& a, const mat3& b);		// m1 * m2
     friend mat3 operator * (const mat3& a, double d);	    // m1 * 3.0
     friend mat3 operator * (double d, const mat3& a);	    // 3.0 * m1
     friend mat3 operator / (const mat3& a, double d);	    // m1 / 3.0
     friend int operator == (const mat3& a, const mat3& b);	    // m1 == m2 ?
     friend int operator != (const mat3& a, const mat3& b);	    // m1 != m2 ?
     friend void Swap(mat3& a, mat3& b);			    // swap m1 & m2

     friend std::istream& operator >> (std::istream& s, mat3& v);
     friend std::ostream& operator << (std::ostream& s, const mat3& v);

    // necessary friend declarations
    friend vec3 operator * (const mat3& a, const vec3& v);	    // linear transform
    friend  mat3 operator * (const mat3& a, const mat3& b);	// matrix 3 product
};

const mat3 identity3D(axisX, axisY, axisZ);
const mat3 zero3D(vec3Zero, vec3Zero, vec3Zero);

class  Quaternion
{
protected:

    double n[4];

public:

    // Constructors
    Quaternion();
    Quaternion(double w, double x, double y, double z);
    Quaternion(const Quaternion& q);

    // Static functions
    static double Dot(const Quaternion& q0, const Quaternion& q1);
    static double Distance(const Quaternion& q0, const Quaternion& q1); // returns angle between in radians
    static Quaternion Exp(const Quaternion& q);
    static Quaternion Log(const Quaternion& q);
    static Quaternion UnitInverse(const Quaternion& q);
    static Quaternion Slerp(const Quaternion& q0, const Quaternion& q1, double t);
    static Quaternion Intermediate (const Quaternion& q0, const Quaternion& q1, const Quaternion& q2);
    static Quaternion Squad(const Quaternion& q0, const Quaternion& a, const Quaternion& b, const Quaternion& q1, double t);
    static Quaternion ProjectToAxis(const Quaternion& q, vec3& axis);

    // Conversion functions
    void ToAxisAngle (vec3& axis, double& angleRad) const;
    void FromAxisAngle (const vec3& axis, double angleRad);

    vec3 ToExpMap() const;
    void FromExpMap(const vec3& expmap);

    mat3 ToRotation () const;
    void FromRotation (const mat3& rot);

    // Assignment operators
    Quaternion& operator = (const Quaternion& q);	// assignment of a quaternion
    Quaternion& operator += (const Quaternion& q);	// summation with a quaternion
    Quaternion& operator -= (const Quaternion& q);	// subtraction with a quaternion
    Quaternion& operator *= (const Quaternion& q);	// multiplication by a quaternion
    Quaternion& operator *= (double d);		// multiplication by a scalar
    Quaternion& operator /= (double d);		// division by a scalar

    // Indexing
    double& W();
    double W() const;
    double& X();
    double X() const;
    double& Y();
    double Y() const;
    double& Z();
    double Z() const;
    double& operator[](int i); // carefull using these, W is last component!
    double operator[](int i) const;

    // Friends
     friend Quaternion operator - (const Quaternion& q);							// -q
     friend Quaternion operator + (const Quaternion& q0, const Quaternion& q1);	    // q0 + q1
     friend Quaternion operator - (const Quaternion& q0, const Quaternion& q1);	// q0 - q1
     friend Quaternion operator * (const Quaternion& q, double d);			// q * 3.0
     friend Quaternion operator * (double d, const Quaternion& q);			// 3.0 * v
     friend Quaternion operator * (const Quaternion& q0, const Quaternion& q1);  // q0 * q1
     friend Quaternion operator / (const Quaternion& q, double d);			// q / 3.0
     friend bool operator == (const Quaternion& q0, const Quaternion& q1);		// q0 == q1 ?
     friend bool operator != (const Quaternion& q0, const Quaternion& q1);		// q0 != q1 ?

     friend std::istream& operator >> (std::istream& s, Quaternion& v);
     friend std::ostream& operator << (std::ostream& s, const Quaternion& v);

    // Special functions
    double Length() const;
    double SqrLength() const;
    Quaternion& Normalize();
    Quaternion Conjugate() const;
    Quaternion Inverse() const;
    void Zero();

    friend mat3;
};


#endif

