// main.cpp
#include <iostream>
#include "physicsengine.hpp"

int main() {
    PhysicsEngine physics;

    RigidBody sphere1({0.0f, 2.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    RigidBody sphere2({3.0f, 0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, 1.0f, 1.0f);

    physics.AddRigidBody(sphere1);
    physics.AddRigidBody(sphere2);

    float deltaTime = 0.1f;

    for (int i = 0; i < 20; ++i) {
        // Apply a force (e.g., user input) to the sphere
        sphere1.ApplyForce({1.0f, 0.0f, 0.0f});

        // Simulate the physics
        physics.Simulate(deltaTime);

        // Print the position of the spheres
        std::cout << "Time Step " << i << ": Sphere1 Position = ("
                  << sphere1.position.x << ", " << sphere1.position.y << ", " << sphere1.position.z << "), "
                  << "Sphere2 Position = (" << sphere2.position.x << ", " << sphere2.position.y << ", "
                  << sphere2.position.z << ")\n";
    }

    return 0;
}
