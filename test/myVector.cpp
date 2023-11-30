#include <cmath>
#include <valarray>

struct Vector2 {
 float x,y;
 Vector2() {
   x = y = 0;
 }
 Vector2(float xx,float yy) {
   x = xx; y = yy;
 }
 Vector2 operator+(const Vector2 &o) {
   return Vector2(x+o.x,y+o.y);
 }
 Vector2& operator+=(const Vector2 &o) {
   x += o.x; y += o.y;
   return (*this);
 }
 Vector2 operator-(const Vector2 &o) {
   return Vector2(x-o.x,y-o.y);
 }
 Vector2 operator-() {
   return Vector2(-x,-y);
 }
 Vector2& operator-=(const Vector2 &o) {
   x -= o.x; y -= o.y;
   return (*this);
 }
 Vector2 operator*(const float f) {
   return Vector2(x*f,y*f);
 }
 Vector2& operator*=(const float f) {
   x *= f; y *= f;
   return (*this);
 }
 float dot(const Vector2 &o) const {
   return x*o.x + y*o.y;
 }
 float length() const {
   return sqrt(x*x+y*y);
 }
 float angle() const {
   return atan2(y,x);
 }
 float angle(const Vector2 &o) {
   return acos(angle(o));
 }
 float cosangle(const Vector2 &o) {
   return dot(o)/( length() * o.length() );
 }
};
Vector2 operator+(const Vector2 &v1, const Vector2 &v2) {
    return Vector2(v1.x+v2.x,v1.y+v2.y);
}
Vector2 operator-(const Vector2 &v1, const Vector2 &v2) {
    return Vector2(v1.x-v2.x,v1.y-v2.y);
}
Vector2 operator*(const Vector2 &v, const float f) {
    return Vector2(v.x*f,v.y*f);
}