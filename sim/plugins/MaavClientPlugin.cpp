#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>

namespace gazebo
{
class MaavClientPlugin : public SystemPlugin
{
public:
    MaavClientPlugin() : set_camera(false) {}

    virtual ~MaavClientPlugin() { this->connections.clear(); }

public:
    void Load(int /*_argc*/, char** /*_argv*/)
    {
        userCam.reset();
        this->connections.push_back(
            event::Events::ConnectPreRender(std::bind(&MaavClientPlugin::Update, this)));
    }

private:
    void Init() {}

private:
    void Update()
    {
        if (set_camera) return;

        if (!this->userCam)
        {
            this->userCam = gui::get_active_camera();
            return;
        }

        std::cout << "[Client Plugin] Got user camera" << std::endl;

        rendering::ScenePtr scene = rendering::get_scene();

        // Wait until the scene is initialized.
        if (!scene || !scene->Initialized()) return;

        std::cout << "[Client Plugin] Scene initialized" << std::endl;

        auto fuselage = scene->GetVisual("MaavMatrice::fuselage::fuselage_visual");
        if (fuselage)
        {
            std::cout << "[Client Plugin] Found fuselage visual" << std::endl;
            ignition::math::Pose3d fuselage_pose = fuselage->WorldPose();
            ignition::math::Vector3d offset = {-12, 0, 5};

            // Set camera pose
            ignition::math::Pose3d camera_pose;
            camera_pose.Pos() = fuselage_pose.Pos() + fuselage_pose.Rot() * offset;

            ignition::math::Vector3d transl_offset = fuselage_pose.Pos() - camera_pose.Pos();
            double pitch = std::atan2(std::abs(offset.Z()), std::abs(offset.X()));
            std::cout << pitch << std::endl;
            camera_pose.Rot() = fuselage_pose.Rot() * ignition::math::Quaterniond(0, pitch, 0);

            std::cout << "[Client Plugin] Setting camera inital pose" << std::endl;
            userCam->SetWorldPose(camera_pose);

            set_camera = true;
        }
    }

private:
    rendering::UserCameraPtr userCam;
    bool set_camera;

    std::vector<event::ConnectionPtr> connections;
};

GZ_REGISTER_SYSTEM_PLUGIN(MaavClientPlugin)
}  // namespace gazebo
