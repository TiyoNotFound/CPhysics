
# CPhysics

CPhysics is a sophisticated open-source C++ library designed for fundamental physics simulations. Developed with versatility and ease of integration in mind, this lightweight framework empowers developers to simulate rigid body dynamics, apply forces, and model basic collisions.

## Features

### 1. **Rigid Body Dynamics**
   - **Realistic Simulation:** Simulate the motion of rigid bodies in three-dimensional space.
   - **Numerical Integration:** Utilize basic numerical integration methods for accurate position and velocity updates.

### 2. **Forces and Movement**
   - **External Forces:** Apply external forces effortlessly, allowing for user input or environmental effects.
   - **Gravity Simulation:** Experience realistic simulations with gravity applied uniformly to all rigid bodies.

### 3. **Collision Detection and Response**
   - **Collision Detection:** Basic yet effective collision detection for spheres.
   - **Collision Response:** Implement a simple collision response mechanism, swapping velocities upon collision.

### 4. **Versatility**
   - **Seamless Integration:** Easily integrate SimplePhysicsEngine into existing C++ projects with minimal dependencies.
   - **Lightweight Design:** Keep your project lightweight with a minimalistic framework.

## Usage

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/TiyoNotFound/CPhysics.git
   ```

2. **Include the Library:**
   Include the `physicsengine.hpp` header file in your C++ project.

3. **Create Instances:**
   Create instances of `RigidBody` and `PhysicsEngine` to initiate and simulate physics.

4. **Customize Parameters:**
   Customize forces, damping, and other parameters as needed for your specific simulation.

## Example

```cpp
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
        sphere1.ApplyForce({1.0f, 0.0f, 0.0f});
        physics.Simulate(deltaTime);

        std::cout << "Time Step " << i << ": Sphere1 Position = ("
                  << sphere1.position.x << ", " << sphere1.position.y << ", " << sphere1.position.z << "), "
                  << "Sphere2 Position = (" << sphere2.position.x << ", " << sphere2.position.y << ", "
                  << sphere2.position.z << ")\n";
    }

    return 0;
}
```

## Advantages

1. **Ease of Integration:**
   - Incorporate the library seamlessly into your C++ projects, ensuring a smooth development process.

2. **Realistic Simulation:**
   - Experience realistic physics simulations with accurate rigid body dynamics and collision handling.

3. **Versatile Application:**
   - Adapt the framework to a wide range of applications, from game development to scientific simulations.

4. **Lightweight Design:**
   - Keep your projects lightweight with a minimalistic design that prioritizes efficiency.
