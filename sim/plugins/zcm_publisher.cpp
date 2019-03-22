#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

namespace gazebo {
class ZcmPublisher : public ModelPlugin {
    /// \brief Constructor
   public:
    ZcmPublisher() { std::cout << "Ground Truth Plugin works!" << std::endl; }

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
   public:
    void OnUpdate() {
        std::cerr << "\nThe ground truth plugin is attached to model ["
                  << model->GetName() << "]\n";
        double x, y, z, qw, qx, qy, qz;
        const ignition::math::Pose3d& pose = model->WorldPose();
        const ignition::math::Vector3<double>& v = pose.Pos();
        const ignition::math::Quaternion<double>& q = pose.Rot();
        x = v.X();  // x coordinate
        y = v.Y();  // y coordinate
        z = v.Z();  // z coordinate
        qw = q.W();
        qx = q.X();
        qy = q.Y();
        qz = q.Z();
        std::cout << "\nX coordinate: " << x << "\nY coordinate: " << y
                  << "\nZ coordinate: " << z;
        std::cout << "\nQuaternion W: " << qw << "\nQuaternion X: " << qx
                  << "\nQuaternion Y: " << qy << "\nQuaternion Z: " << qz;
    }
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        model = _model;
        sdf = _sdf;
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&GroundTruthPlugin::OnUpdate, this));
    }

   private:
    physics::ModelPtr model;
    sdf::ElementPtr sdf;

   private:
    event::ConnectionPtr updateConnection;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(GroundTruthPlugin)

}  // namespace gazebo
