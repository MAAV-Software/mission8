#ifndef __PLANE_FITTER_HPP_MAAV_VISION__
#define __PLANE_FITTER_HPP_MAAV_VISION__

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace maav::vision
{
class PlaneFitter
{
public:
    /**
     * \param inlierThreshold max distance of inliers to the fitted plane
     * Inlier threshold must be passed in to initialize
     * PCL's RANSAC plane fitter used under the hood
     */
    explicit PlaneFitter(const float inlier_threshold);
    /** \param cloud point cloud pointer,
     * \return plane coefficients
     *
     * will provide plane coefficients on success
     * On failure will return zero dimension matrix
     */
    Eigen::MatrixXf fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    /** \brief Calculate the plane information without returning coefficients
     * \return boolean that indicates whether plane fitting was successful
     * \param cloud a point cloud ptr to the input point cloud
     * \param zdot reference that will be set to the change in height
     * \param zdepth refrence that will be set to the depth to the
     * fitted plane
     * \param pitch reference that will be set to the pitch of the quad
     * \param roll reference that will be set to the roll of the quad
     *
     * Computes the change in height of the quadcopter, depth to the
     * plane of the quadcopter, roll or the quadcopter, and pitch of
     * the quadcopter through using PCL RANSAC plane fitting to fit
     * cloud to a plane, and assuming that that plane is the ground
     * plane, computing peices of the quadcopter's pose
     */
    bool runPlaneFitting(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float &zdot,
        float &zdepth, float &roll, float &pitch);
    /** \brief Obtain all the plane information needed by driver
     * \return boolean that indicates whether plane fitting was successful
     * \param cloud a point cloud ptr to the input point cloud
     * \param zdot reference that will be set to the change in height
     * \param zdepth refrence that will be set to the depth to the
     * fitted plane
     * \param pitch reference that will be set to the pitch of the quad
     * \param roll reference that will be set to the roll of the quad
     * \param coefs reference that will be set to the plane coefficients
     * of the plane that cloud will be fit to
     *
     * Obtain all the plane information needed by driver
     * zdot is change in height since last measurement
     * zdepth is distance to plane along the line going through
     * the top of the quad (varies with roll and pitch)
     * roll is the same as quadcopter roll, pitch is the same
     * as quadcopter pitch, coefs is the matrix that can be passed
     * in to hold the plane coefficients computed by the class
     */
    bool getPlaneInfo(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float &zdot, float &zdepth,
        float &roll, float &pitch, Eigen::MatrixXf &coefs);
    /** Get the last computed height
     * \return the last computed quadcopter height
     */
    float getLastHeight() const { return last_height_; }
private:
    const float inlier_thresh_;
    float last_height_;
    Eigen::MatrixXf junk_matrix_;
};  // PlaneFitter
}
#endif
