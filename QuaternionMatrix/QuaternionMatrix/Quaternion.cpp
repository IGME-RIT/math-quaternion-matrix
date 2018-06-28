#include "Quaternion.h"

Quaternion::Quaternion()
	: w(0), x(0), y(0), z(0)
{
}

Quaternion::Quaternion(float w, float x, float y, float z)
	: w(w), x(x), y(y), z(z)
{
}

Quaternion::Quaternion(float w, Vector3D v)
	: w(w), x(v.x), y(v.y), z(v.z)
{
}

Quaternion operator+(Quaternion q, Quaternion r)
{
	return Quaternion(q.w + r.w, q.x + r.x, q.y + r.y, q.z + r.z);
}

Quaternion operator-(Quaternion q)
{
	return Quaternion(-q.w, -q.x, -q.y, -q.z);
}

Quaternion operator-(Quaternion q, Quaternion r)
{
	return q + (-r);
}

// Since we can represent Quaternions as ordered pair
// q = [sa, a] r = [sb, b]
// qr = [sa, a]*[sb, b]
// qr = (sa + xai + yaj + zak) * (sb + xbi + ybj + zbk)
// qr = (sasb - xaxb - yayb - zazb) +
//		(saxb + sbxa + yazb - ybza) i +
//		(sayb + sbya + zaxb - zbxa) j +
//		(sazb + sbza + xayb - xbya) k
Quaternion operator*(Quaternion q, Quaternion r)
{
	float wComp = (q.w * r.w) - (q.x * r.x) - (q.y * r.y) - (q.z * r.z);
	float xComp = (q.w * r.x) + (q.x * r.w) - (q.y * r.z) + (q.z * r.y);
	float yComp = (q.w * r.y) + (q.x * r.z) + (q.y * r.w) - (q.z * r.x);
	float zComp = (q.w * r.z) - (q.x * r.y) - (q.y * r.x) + (q.z * r.w);

	return Quaternion(wComp, xComp, yComp, zComp);
}

Quaternion operator*(float s, Quaternion q)
{
	return Quaternion(s*q.w, s*q.x, s*q.y, s*q.z);
}

Quaternion operator*(Quaternion q, float s)
{
	return s * q;
}

// The norm is the sum of the squares of all elements of the Quaternion
float Norm(Quaternion q)
{
	return (q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
}

// The magnitude is the obtained the getting the square root of the norm of the Quaternion
float Magnitude(Quaternion q)
{
	return (sqrt(Norm(q)));
}

Quaternion operator/(Quaternion q, float s)
{
	return Quaternion(q.w / s, q.x / s, q.y / s, q.z / s);
}

// The division between two Quaternion is obtained by
// calculating the norm of the divisor
// multiplying the two quaternions and
// dividing the resultant quaternion by norm of the divisor
Quaternion operator/(Quaternion q, Quaternion r)
{
	float norm = Norm(r);

	Quaternion x = q * r;

	return Quaternion(x / norm);
}

// Normalize the Quaternion by dividing it with the magnitude
Quaternion Normalize(Quaternion q)
{
	return (q / Magnitude(q));
}

// The Conjugate of the Quaternion is obtained by negating the imaginary part of the Quaternion
Quaternion Conjugate(Quaternion q)
{
	return Quaternion(q.w, -q.x, -q.y, -q.y);
}

// The inverse of a quaternion is obtained by dividing the Conjugate with the Norm of the Quaternion
Quaternion Inverse(Quaternion q)
{
	return(Conjugate(q) / Norm(q));
}

// Similar to vector dot products, the quaternion dot products are calculated by
// multiplying corresponding scalar parts and summing them up.
float Dot(Quaternion q, Quaternion r)
{
	return ((q.w*r.w) + (q.x*r.x) + (q.y*r.y) + (q.z*r.z));
}

// To calculate the angle between two quaternions
// Obtain the cosine of the angle by calculating the dot product of two quaternions
// and dividing it by the product of the magnitude of both the quaternions
// calculate the cosine inverse of the result to obtain angle
float AngleBetweenQuaternions(Quaternion q, Quaternion r)
{
	float cosOfAngle = Dot(q, r) / (Magnitude(q) * Magnitude(r));

	return (acos(cosOfAngle));
}

// The Quaternion for the rotation is obtained by
// creating a quaternion from the vector to get direction and the angle provided
Quaternion Rotation(Vector3D v, float a)
{
	v = Normalize(v);

	return Quaternion(cos(a / 2), (sin(a / 2) * v));
}

// Rotation matrix created from quaternion, when multiplied with the Vector3D returns
// a rotated vector along the given quaternion
Matrix3D RotationMatrix(Quaternion q)
{
	float n00 = 1 - (2 * q.y*q.y) - (2 * q.z*q.z);
	float n01 = 2 * ((q.x * q.y) + (q.w * q.z));
	float n02 = 2 * ((q.x * q.z) - (q.w * q.y));
	float n10 = 2 * ((q.x * q.y) - (q.w * q.z));
	float n11 = 1 - (2 * q.x*q.x) - (2 * q.z*q.z);
	float n12 = 2 * ((q.y * q.z) + (q.w * q.x));
	float n20 = 2 * ((q.x * q.z) + (q.w * q.y));
	float n21 = 2 * ((q.y * q.z) - (q.w * q.x));
	float n22 = 1 - (2 * q.x*q.x) - (2 * q.y*q.y);

	return Matrix3D(n00, n01, n02,
		n10, n11, n12,
		n20, n21, n22);
}

// Returns the rotated vector using the given quaternion 
Vector3D RotateVector(Vector3D v, Quaternion q)
{
	Matrix3D rotationMatrix = RotationMatrix(q);

	return rotationMatrix * v;
}

// Prints out the quaternion in the format (w, x, y, z)
std::ostream& operator<<(std::ostream& os, Quaternion q)
{
	os << "(" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << ")";
	return os;
}