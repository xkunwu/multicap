#include "rMainEngine.h"
#include "rFreenect2CameraFactory.h"

int main(int argc, char *argv[])
{
    std::unique_ptr<rec3D::rMainEngine> engine = std::make_unique<rec3D::rMainEngine>(
                rec3D::rMainEngine::Type::LIBFREENECT2,
                1);

    std::unique_ptr<rec3D::rCameraFactory> kinect_factory =
            std::make_unique<rec3D::rFreenect2CameraFactory>();

    bool active = engine->find(std::move(kinect_factory));

    engine->capture_all();
    engine->save_data();
}
