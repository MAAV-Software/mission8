#ifndef MAAV_GCS_STATUS_FRAME_HPP
#define MAAV_GCS_STATUS_FRAME_HPP

#include "GCSConsts.hpp"
#include "GlibZCM.hpp"
#include "common/utils/debug.hpp"

#include "Messages.hpp"

#include <gtkmm/box.h>
#include <gtkmm/frame.h>
#include <gtkmm/label.h>
#include <stdlib.h>
#include <iomanip>
#include <sstream>

namespace maav
{
namespace gcs
{
/**
 * @brief The GCS status frame
 *
 * @details This frame displays which subsystems are currently online (based on
 * the amount of time that's passed since the last message that each sent)
 */
class StatusFrame : public Gtk::Frame
{
	// the four possible statuses
	//(un)calibrated is useful only for the camera status
	enum class Status
	{
		online,
		quiet,
		down,
		calibrated,
		uncalibrated
	};

	// retrieves the (marked up) text to display for each status
	static const std::string status_text(Status, const char* = "");

	template <typename Message>
	class StatusDisplay : public Gtk::Box, GlibZCM::Handler<Message>
	{
	   public:
		// creates a status display on a given channel with some particular
		// subsystem label text
		StatusDisplay(GlibZCM& zcm, const char* channel, const char* name_text,
					  const GCSConsts& consts_in)
			: GlibZCM::Handler<Message>(zcm, channel),
			  name{name_text},
			  cur_status{Status::uncalibrated},
			  CONSTS{consts_in}

		{
			update_status(Status::down);
			set_spacing(CONSTS.MED_SPACE);
			add(name), add(status);
		}

		virtual ~StatusDisplay() { timer_connection.disconnect(); }
	   protected:
		// overrides the message-handling virtual function to reset the state to
		// online whenever a message is received
		virtual void on_message(const Message&) override { update_status(Status::online); }
		// the two components of the display
		Gtk::Label name;
		Gtk::Label status;

		// the current status that this display is showing
		Status cur_status;

		// Constants container
		const GCSConsts& CONSTS;

	   private:
		// a connection which is used to control the timer callback
		sigc::connection timer_connection;

		// upates the current status and resets the timer if needed
		virtual void update_status(Status new_status)
		{
			cur_status = new_status;
			status.set_markup(status_text(cur_status));

			if (cur_status == Status::quiet)
			{
				MAAV_DEBUG("%s quiet!", name.get_text().c_str());
			}
			else if (cur_status == Status::down)
			{
				MAAV_DEBUG("%s down!", name.get_text().c_str());
			}

			// the timer needs to be reset for every status that isn't down
			if (new_status != Status::down)
			{
				// this new timer setting should override any previous one
				timer_connection.disconnect();

				// the timer is set to invoke a callback after (around) the
				// corresponding timeout
				// if the state is updated before the timer fires, the above line
				// will make sure the callback isn't run anyway
				timer_connection = Glib::signal_timeout().connect_seconds(
					[this]() {
						update_status(cur_status == Status::online ? Status::quiet : Status::down);
						return false;
					},
					cur_status == Status::online ? CONSTS.QUIET_TIMEOUT : CONSTS.DOWN_TIMEOUT);
			}
		}
	};

	class PlannerDisplay : public StatusDisplay<waypoint_t>
	{
	   public:
		PlannerDisplay(GlibZCM& zcm, const char* channel, const GCSConsts& consts_in)
			: StatusDisplay<waypoint_t>(zcm, channel, "Current Mission:", consts_in)
		{
			update_status(Status::down, "No Mission Running");
		}

		virtual ~PlannerDisplay() {}
	   protected:
		virtual void on_message(const waypoint_t& msg) override
		{
			std::string mode_str;
			std::stringstream ss;
			switch (msg.pmode)
			{
				case 0:
					mode_str = "Autonomous";
					break;
				case 1:
					mode_str = "Hover";
					break;
				case 2:
					mode_str = "Takeoff";
					break;
				case 3:
					mode_str = "Land";
					break;
				case 4:
					if (msg.mode == 1 || msg.mode == 3)
					{
						ss << std::setprecision(3) << "Pose: (" << msg.pose[0] << ", "
						   << msg.pose[1] << ", " << -msg.pose[2] << ", " << msg.pose[3] << ")  ";
					}
					if (msg.mode == 2 || msg.mode == 3)
					{
						ss << std::setprecision(3) << "Rate: (" << msg.pose[0] << ", "
						   << msg.pose[1] << ", " << msg.pose[2] << ", " << msg.pose[3] << ")";
					}
					mode_str = ss.str();
					break;
				case 5:
					mode_str = "Exit Arena";
					break;
				default:
					mode_str = "No Mission Running";
			}
			update_status(Status::online, mode_str.c_str());
		}

	   private:
		void update_status(Status, const char* status_text)
		{
			status.set_markup(std::string{"<tt>"} + status_text + "</tt>");
		}
	};

	class CameraDisplay : public StatusDisplay<camera_disc_t>
	{
	   public:
		CameraDisplay(GlibZCM& zcm, const char* channel, const GCSConsts& consts_in)
			: StatusDisplay<camera_disc_t>(zcm, channel, "Cameras:", consts_in)
		{
			update_status(Status::uncalibrated);
		}

		virtual ~CameraDisplay() {}
	   protected:
		virtual void on_message(const camera_disc_t& msg) override
		{
			std::stringstream ss;
			ss << (int)msg.numCameras;
			MAAV_DEBUG("CAMERA MESSAGE: %d %s", msg.numCameras, ss.str().c_str());
			update_status(Status::calibrated, ss.str().c_str());
		}

	   private:
		void update_status(Status new_status, const char* extra = "")
		{
			cur_status = new_status;
			status.set_markup(status_text(cur_status, extra));

			if (cur_status == Status::uncalibrated)
			{
				MAAV_DEBUG("%s uncalibrated!", name.get_text().c_str());
			}
			else if (cur_status == Status::calibrated)
			{
				MAAV_DEBUG("%s calibrated!", name.get_text().c_str());
			}
		}
	};

	class ControllerDisplay : public StatusDisplay<waypoint_t>
	{
	   public:
		ControllerDisplay(GlibZCM& zcm, const char* channel, const GCSConsts& consts_in)
			: StatusDisplay<waypoint_t>(zcm, channel, "Controller Target:", consts_in)
		{
			update_status(Status::down, "No Mission Running");
		}

		virtual ~ControllerDisplay() {}
	   protected:
		virtual void on_message(const waypoint_t& msg) override
		{
			std::string mode_str;
			std::stringstream ss;
			ss << std::setprecision(3) << "Pose: (" << msg.pose[0] << ", " << msg.pose[1] << ", "
			   << -msg.pose[2] << ", " << msg.pose[3] << ")  ";
			ss << "Rate: (" << msg.pose[0] << ", " << msg.pose[1] << ", " << msg.pose[2] << ", "
			   << msg.pose[3] << ")";
			mode_str = ss.str();
			update_status(Status::online, mode_str.c_str());
		}

	   private:
		void update_status(Status, const char* status_text)
		{
			status.set_markup(std::string{"<tt>"} + status_text + "</tt>");
		}
	};

	// Constants container
	const GCSConsts& CONSTS;

	// the set of boxes for holding the status displays
	// a set of nested boxes is used for each row in order to center them
	Gtk::Box outer_box{Gtk::ORIENTATION_VERTICAL};
	Gtk::Box upper_cbox, lower_cbox, under_cbox, info_cbox;
	Gtk::Box upper_box, lower_box, under_box, info_box{Gtk::ORIENTATION_VERTICAL};

	// the set of status displays to show
	StatusDisplay<nav_runstate_t> path_planner;
	StatusDisplay<state_t> localization;
	StatusDisplay<nav_runstate_t> line_det;
	StatusDisplay<nav_runstate_t> roomba_det;
	StatusDisplay<obstacle_list_t> obstacle_det;
	StatusDisplay<waypoint_t> controller;
	CameraDisplay camera_cal;
	PlannerDisplay planner_info;
	ControllerDisplay controller_info;

   public:
	/**
	 * @brief Sets up a status frame
	 * @param zcm The GlibZCM that this status frame should use to monitor
	 * subsystem statuses
	 */
	StatusFrame(GlibZCM& zcm, const GCSConsts& consts_in);
};
}
}

#endif
