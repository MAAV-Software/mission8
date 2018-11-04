#ifndef MAAV_GCS_STAT_CMD_HPP
#define MAAV_GCS_STAT_CMD_HPP

#include "GlibZCM.hpp"
#include "common/utils/debug.hpp"

#include <string>
#include <thread>
#include <utility>

namespace maav
{
namespace gcs
{
/**
 * @brief A status/command handler
 *
 * @details This type adapts a GlibZCM handler to provide a convenient
 * implementation of the status/command protocol used by GCS. This goal of this
 * protocol is to allow remote processes to monitor and request updates to a
 * particular piece of state held by another process. For a particular status
 * type my_status_t, this is accomplished via two ZCM channels, MY_STATUS_STAT
 * and MY_STATUS_CMD, which carry status ("stat") and command ("cmd") messages
 * respectively. Processes can take on one of two roles in this protocol:
 *
 * ### The Owning Process
 *
 * Exactly one process for each piece of state managed by this protocol serves
 * as the "owning process;" this process is the one that directly uses the state
 * and the one that is responsible for maintaining it, and will usually be run
 * on the vehicle. Given the my_status_t example mentioned above, this process
 * must do two things:
 *
 * * periodically publish its current state on MY_STATUS_STAT at a regular, but
 * relatively infrequent, interval (~2Hz is a good value)
 * * subscribe to MY_STATUS_CMD and update its state when new command messages
 * are received
 *
 * This is pretty simple to implement, and some basic code might look like this:
 *
 * ```
 * mutex lock;
 * my_status_t state;
 *
 * //register for command messages and handle state updates
 * register_zcm_handler("MY_STATUS_CMD", [&](const my_status_t& msg) {
 *     lock_guard<mutex> g{lock};
 *     do_state_transition(state, msg);
 *     state = msg;
 * });
 *
 * //periodically broadcast the current state
 * thread t {[&]() {
 *     while (true) {
 *         {
 *             lock_guard<mutex> g{lock};
 *             publish_zcm_message("MY_STATUS_STAT", state);
 *         }
 *         this_thread::sleep_for(500ms);
 *     }
 * }};
 *
 * while (not zcm.handle());
 *
 * t.join();
 * ```
 *
 * ### Remote Processes
 *
 * These processes do not directly manage the state, but listen for updates from
 * the owning process and can send commands to perform updates; in most cases,
 * this end of the protocol makes up part of GCS, and this is what StatCmd
 * implements. For the same my_status_t example, this process must:
 *
 * * subscribe to MY_STATUS_STAT for status updates and track changes to the
 * owning processes state through the status messages
 * * when requesting an update, publish a command message containing the new
 * state on MY_STATUS_CMD and then re-send the command message every time a
 * status message is received on MY_STATUS_STAT until the status has been
 * updated to match the new status that has been set
 *
 * StatCmd takes care of most of these details, so some sample code might look
 * like this:
 *
 * ```
 * class MyStatusControl
 * {
 *     StatCmd<my_status_t> my_status;
 *
 *     //...
 *
 * public:
 *
 *     MyStatusControl(GlibZCM& zcm) : my_status{zcm, "MY_STATUS"}
 *     {
 *         //this signal handler is called when status updates are received or
 *         //when the status is updated via cmd
 *         //here, it should update the UI to reflect the new status
 *         my_status.signal_update().connect([this](const my_status_t& stat) {
 *             set_ui_status(stat);
 *         });
 *     }
 *
 *     //this might be a function that gets called when a button is pressed to
 *     //transmit the current status
 *     //cmd automatically handles the status update and any required resends
 *     void send_ui_status()
 *     {
 *         my_stats.cmd(get_ui_status());
 *     }
 * };
 * ```
 *
 * More examples of good StatCmd usage are available throughout the maav::gcs
 * code.
 */
template <typename Status>
class StatCmd : public GlibZCM::Handler<Status>
{
    // a convenient alias for the long parent class name
    using super = GlibZCM::Handler<Status>;

    // the current reported status, old_stat, and the desired status, new_stat
    Status old_stat, new_stat;

    // which channel to use to send command messages
    std::string cmd_channel;

    // the signal for status updates
    sigc::signal<void, const Status&> update_signal;

    // is the corresponding command channel primed
    bool primed{false};

    // how to update the desired status and correctly emit the signal
    void update_new(const Status& stat)
    {
        update_signal.emit(stat);
        new_stat = stat;
    }

    protected:
    /**
     * @brief Handles new messages and updates the internal status
     * @param msg The new message that has come in
     */
    void on_message(const Status& msg) override
    {
        // if there's an update in progress and the status still hasn't been
        // updated, resend the command message
        if (old_stat != new_stat)
        {
            if (msg != new_stat) super::get_zcm().publish(cmd_channel, new_stat);
        }

        // otherwise, if there isn't an update in progress, update the desired
        // message if there's a new status
        else if (msg != new_stat)
            update_new(msg);

        // always update the reported status
        old_stat = msg;

        // call back up to emit the message signal
        super::on_message(msg);
    }

    public:
    /**
     * @brief Creates a status/command handler
     * @param zcm The GlibZCM instance to use for communicating with ZCM
     * @param channel The channel prefix to use for status and command messages:
     * "_CMD" is appended to get the command channel and "_STAT" is appended for
     * the status channel
     * @param init The initial status value
     */
    StatCmd(GlibZCM& zcm, const std::string& channel, const Status& init = Status())
        : super{zcm, channel + "_STAT"},
          old_stat{init},
          new_stat{init},
          cmd_channel{channel + "_CMD"}
    {
    }

    /**
    * @brief Alternate constructor enabling explicit specification of command
    * and status channels.
    *
     * @param zcm The GlibZCM instance to use for communicating with ZCM
     * @param cnd_channel_in The command channel
     * @param stat_channel_on The status channel
     * @param init The initial status value
    */
    StatCmd(GlibZCM& zcm, const std::string& cmd_channel_in, const std::string& stat_channel_in,
        const Status& init = Status())
        : super{zcm, stat_channel_in}, old_stat{init}, new_stat{init}, cmd_channel{cmd_channel_in}
    {
    }

    /**
    * @ brief Prime the corresponding command channel. This actually primes the
    * ipc channel on gcsClient since udpm does not need to be primed.
    */
    void prime()
    {
        if (!primed)
        {
            super::get_zcm().publish(cmd_channel, new_stat);
            primed = true;
        }
    }

    /**
    * @brief Move constructor
    *
    * @param other The other StatCmd to move data from
    */
    StatCmd(StatCmd<Status>&& other) noexcept
    {
        old_stat = std::move(other.old_stat);
        new_stat = std::move(other.new_stat);
        cmd_channel = std::move(other.cmd_channel);
        update_signal = std::move(other.update_signal);
        primed = other.primed;
    }

    /**
    * @brief Move assignment operator
    *
    * @param other The other StatCmd to move data from
    *
    * @return This StatCmd
    */
    StatCmd<Status>& operator=(StatCmd<Status>&& other) noexcept
    {
        if (this != &other)
        {
            old_stat = std::move(other.old_stat);
            new_stat = std::move(other.new_stat);
            cmd_channel = std::move(other.cmd_channel);
            update_signal = std::move(other.update_signal);
            primed = other.primed;
        }
        return *this;
    }

    /**
    * @brief Sets initial old_stat and new_stat to the given Status
    *
    * @param stat_in The Status to set old_stat and new_stat to
    */
    void initStat(Status stat_in)
    {
        new_stat = stat_in;
        old_stat = stat_in;
    }

    /**
     * @brief Retreives the current status, which will be the one set by cmd if an
     * update is in progress
     * @return The current status
     */
    const Status& stat() const { return new_stat; }
    /**
     * @brief Updates the status and starts sending command messages
     * @param cmd The value to update the status to
     */
    void cmd(const Status& cmd)
    {
        //	if (cmd == new_stat) return;
        update_new(cmd);
        super::get_zcm().publish(cmd_channel, new_stat);
    }

    /**
    * @brief Updates the status without sending command sessages
    *
    * @param update The current status
    */
    void update(const Status& update)
    {
        if (update == new_stat) return;
        update_new(update);
    }

    /**
    * @brief Publishes the most recent command message
    */
    void publish_new() { super::get_zcm().publish(cmd_channel, new_stat); }
    /**
     * @brief Exposes the status update signal
     * @return The signal; connect to this to get status updates
     *
     * @details The handler here is called just before the StatCmd is actually
     * updated, so the status value from calling stat() on the StatCmd within the
     * handler will be the old value while the parameter which is passed in is the
     * new one. This behavior is very helpful for implementing UI elements that
     * should track the status if they aren't changed but stop tracking if they
     * are:
     *
     * ```
     * my_status.signal_update().connect([this](const my_status_t& new_stat) {
     *
     *     //the old status is available here, so it can be used to see if the ui
     *     //is matching my_status before the update takes place
     *     const auto old_stat = my_status.stat();
     *     if (get_ui_status() == old_stat) {
     *
     *         //the new status passed as a parameter can then be used to make the
     *         //UI track the status update
     *         set_ui_status(new_stat);
     *     }
     * });
     * ```
     */
    sigc::signal<void, const Status&>& signal_update() { return update_signal; }
};
}
}

#endif
