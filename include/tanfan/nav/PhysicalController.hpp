#ifndef MAAV_PHYSICAL_CONTROLLER_HPP
#define MAAV_PHYSICAL_CONTROLLER_HPP

#include <functional>
#include <string>
#include "../messaging/DataLink.hpp"
#include "Controller.hpp"
#include "Log.hpp"
#include "SerialTTY.hpp"
#include "data_link.h"
#include "msg/feedback_t.h"
#include "msg/gains_t.h"

namespace maav
{
/**
 * @brief A non-blocking interface to talk to the actual controller
 *
 * @details process() should be run in a separate thread. It loops internally,
 * so the body of the thread should call the function outside of any loops.
 */
class PhysicalController : public Controller
{
    public:
    /**
     * @brief Does the needful
     */
    PhysicalController();

    ~PhysicalController();

    /**
     * @brief Connects on the specified port
     */
    void connect(const std::string &portPath);

    /**
     * @brief Disconnects from the controller if connected
     */
    void disconnect() noexcept;

    /**
     * @brief Move the vehicle. Duh.
     */
    virtual void move(double dx, double dy, double dz) override;

    /**
     * @brief Rotate the vehicle.
     */
    virtual void rotate(double dr, double dp, double dy) override;

    /**
     * @brief Fly!
     */
    virtual void takeoff() override;

    /**
     * @brief stop flying :(
     */
    virtual void land() override;

    /**
     * @brief Call to block, receive, and process
     */
    void process(DataLink &dlink);

    /**
     * @brief handles the controller's hot garbage
     */
    void setFeedbackHandler(std::function<void(const feedback_t *)>);

    /**
     * @brief stop this thing
     */
    void stop();

    /**
     * @brief always sends utime of now
     */
    void sendSetpoint(float x, float y, float z, float h, int8_t flags);

    /**
     * @brief Forwards a gains_t
     */
    void sendGains(const gains_t *g);

    /**
     * @brief Forwards a dji_t
     */
    void sendDji(float roll, float pitch, float yaw, float thrust);

    private:
    std::function<void(const feedback_t *)> handler;
    maav::Log::Logger log;
    maav::SerialTTY port;
    bool running;
};

}  // namespace maav

#endif  // MAAV_PHYSICAL_CONTROLLER_HPP
