// physicsengine.cpp
#include "physicsengine.hpp"
#include <iostream>

void RigidBody::ApplyForce(const Vector3& externalForce) {
    force = force + externalForce;
}

void RigidBody::Update(float deltaTime) {
    // Calculate acceleration using Newton's second law (F = ma)
    Vector3 acceleration = force * (1.0f / mass);

    // Update velocity using acceleration and time step
    velocity = velocity + acceleration * deltaTime;

    // Update position using velocity and time step
    position = position + velocity * deltaTime;

    // Reset force for the next simulation step
    force = {0.0f, 0.0f, 0.0f};
}

PhysicsEngine::PhysicsEngine() {}

void PhysicsEngine::AddRigidBody(const RigidBody& rigidBody) {
    rigidBodies.push_back(rigidBody);
}

void PhysicsEngine::Simulate(float deltaTime) {
    // Apply gravity to all rigid bodies
    ApplyGravity({0.0f, -9.8f, 0.0f});

    // Apply damping to all rigid bodies
    ApplyDamping(0.98f);

    // Check for collisions
    CheckCollisions();

    // Resolve collisions
    ResolveCollisions();

    // Update all rigid bodies
    for (auto& body : rigidBodies) {
        body.Update(deltaTime);
    }
}

void PhysicsEngine::ApplyGravity(const Vector3& gravity) {
    for (auto& body : rigidBodies) {
        body.ApplyForce(gravity * body.mass);
    }
}

void PhysicsEngine::ApplyDamping(float dampingFactor) {
    for (auto& body : rigidBodies) {
        body.velocity = body.velocity * dampingFactor;
    }
}

bool PhysicsEngine::CheckSphereCollision(const RigidBody& sphere1, const RigidBody& sphere2) {
    float distanceSquared = pow(sphere1.position.x - sphere2.position.x, 2) +
                            pow(sphere1.position.y - sphere2.position.y, 2) +
                            pow(sphere1.position.z - sphere2.position.z, 2);

    float sumOfRadiiSquared = pow(sphere1.radius + sphere2.radius, 2);

    return distanceSquared <= sumOfRadiiSquared;
}

void PhysicsEngine::CheckCollisions() {
    collisions.clear();

    for (size_t i = 0; i < rigidBodies.size(); ++i) {
        for (size_t j = i + 1; j < rigidBodies.size(); ++j) {
            if (CheckSphereCollision(rigidBodies[i], rigidBodies[j])) {
                collisions.emplace_back(&rigidBodies[i], &rigidBodies[j]);
            }
        }
    }
}

void PhysicsEngine::ResolveCollisions() {
    for (const auto& collision : collisions) {
        // Simple collision response: Swap velocities
        Vector3 tempVelocity = collision.body1->velocity;
        collision.body1->velocity = collision.body2->velocity;
        collision.body2->velocity = tempVelocity;
    }
}
