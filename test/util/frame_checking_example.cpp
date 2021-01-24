#include "wave/geometry/geometry.hpp"

int main() {
    struct BodyFrame;
    struct CameraFrame;
    struct WorldFrame;

    wave::RotationMFd<WorldFrame, BodyFrame> r1;
    wave::RotationMFd<CameraFrame, BodyFrame> r2;

    // Let's get the rotation between World and Camera (maybe)
    // wave::RotationMFd<WorldFrame, CameraFrame> result = r1 * r2;        // fails
    wave::RotationMFd<WorldFrame, CameraFrame> result = r1 * inverse(r2);  // ok
    static_cast<void>(result);
}
