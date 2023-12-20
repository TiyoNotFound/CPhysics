// physicsengine.hpp
#ifndef PHYSICSENGINE_HPP
#define PHYSICSENGINE_HPP

#include <vector>

struct Vector3 {
    float x, y, z;

    Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vector3 operator+(const Vector3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    Vector3 operator*(float scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }
};

struct RigidBody {
    Vector3 position;
    Vector3 velocity;
    Vector3 force;
    float mass;
    float radius;

    RigidBody(const Vector3& position, const Vector3& velocity, float mass, float radius)
        : position(position), velocity(velocity), force({0.0f, 0.0f, 0.0f}), mass(mass), radius(radius) {}

    void ApplyForce(const Vector3& externalForce);
    void Update(float deltaTime);
};

struct Collision {
    RigidBody* body1;
    RigidBody* body2;

    Collision(RigidBody* b1, RigidBody* b2) : body1(b1), body2(b2) {}
};

class PhysicsEngine {
public:
    PhysicsEngine();

    void AddRigidBody(const RigidBody& rigidBody);

    void Simulate(float deltaTime);

    void ApplyGravity(const Vector3& gravity);
    void ApplyDamping(float dampingFactor);

    // New: Basic collision detection and response
    void CheckCollisions();
    void ResolveCollisions();

private:
    std::vector<RigidBody> rigidBodies;
    std::vector<Collision> collisions;

    // New: Helper function to check collision between two spheres
    bool CheckSphereCollision(const RigidBody& sphere1, const RigidBody& sphere2);
};

#endif // PHYSICSENGINE_HPP
