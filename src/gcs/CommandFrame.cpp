#include "gcs/CommandFrame.hpp"
#include <chrono>
#include "common/utils/debug.hpp"

using namespace std;
using namespace std::chrono;
using namespace std::literals::string_literals;

namespace maav
{
namespace gcs
{
CommandFrame::NumberEntry::NumberEntry(bool allow_floats_in) : allow_floats{allow_floats_in} {}
void CommandFrame::NumberEntry::on_insert_text(const Glib::ustring& text, int* position)
{
	if (is_numeral(text)) Gtk::Entry::on_insert_text(text, position);
}

bool CommandFrame::NumberEntry::is_numeral(const Glib::ustring& text)
{
	// prevents segfault
	if (get_text().length() >= 15) return false;

	// only allow one dot
	unsigned num_dots = std::count(get_text().begin(), get_text().end(), '.');

	// validate input
	for (unsigned i = 0; i < text.length(); ++i)
	{
		if (not Glib::Unicode::isdigit(text[i]))
		{
			if (allow_floats && text[i] == '.' && not num_dots)
				++num_dots;
			else
				return false;
		}
	}
	return true;
}

CommandFrame::CommandFrame(GlibZCM& zcm, const GCSConsts& consts_in)
	: Gtk::Frame("Command"),
	  CONSTS{consts_in},
	  runstate{zcm, NAV_RUNSTATE_CMD, "RUNSTATE_NULL"},
	  vision_start{zcm, START_VISION, "VISION_START_NULL"},
	  waypt{zcm, PLANNER_CMD, "PLANNER_NULL"},
	  vision{zcm, CAMERA_DISC_CMD, "CAMERA_NULL"},
	  idle{zcm, IDLE_CHANNEL, "IDLE_STAT"},
	  analog_ctrl{zcm, "DJI", "CTRL_NULL"},
	  xcontroller({0.5, 0.5, 0.5, 0.5}, 0.0)
{
	MAAV_DEBUG("Initializing Command Frame");
	// prime ipc channels
	runstate.prime();
	waypt.prime();
	vision.prime();
	idle.prime();
	vision_start.prime();
	std::this_thread::sleep_for(std::chrono::milliseconds(150));

	// Set default messages
	waypoint_t way;
	way.pmode = 0;
	way.mode = 0;
	fill(begin(way.pose), end(way.pose), 0.);
	fill(begin(way.rate), end(way.rate), 0.);
	waypt.initStat(way);

	nav_runstate_t state;
	state.running_mission = false;
	runstate.initStat(state);

	set_label_align(Gtk::ALIGN_CENTER);

	outer_box.set_border_width(CONSTS.MED_SPACE);
	outer_box.set_spacing(CONSTS.MED_SPACE);

	mission_btn.set_tooltip_text("Start autonomous mission");
	idle_btn.set_tooltip_text("Idle (arm) the vehicle");
	analog_ctrl_sw.set_tooltip_text("Toggle quadcopter control using analog stick controller");

	bbox_top.add(mission_btn);
	bbox_top.add(idle_btn);
	outer_box.add(bbox_top);

	outer_box.add(sep_top);

	set_pose.join_group(autonomous);
	takeoff.join_group(autonomous);
	land.join_group(autonomous);
	hover.join_group(autonomous);
	exit_arena.join_group(autonomous);
	hover.set_active();
	autonomous.set_sensitive(false);
	set_pose.set_sensitive(true);
	takeoff.set_sensitive(true);
	land.set_sensitive(true);
	hover.set_sensitive(true);
	exit_arena.set_sensitive(true);
	mission_btn.set_sensitive(false);
	cmd_btn.set_sensitive(false);

	bbox_middle_right.set_valign(Gtk::ALIGN_CENTER);
	bbox_middle_left.add(autonomous);
	bbox_middle_left.add(takeoff);
	bbox_middle_left.add(land);
	bbox_middle_left.add(hover);
	bbox_middle_left.add(set_pose);
	bbox_middle_left.add(exit_arena);
	bbox_middle_right.add(chk_pose);
	bbox_middle_right.add(chk_rate);
	bbox_middle.add(bbox_middle_left);
	bbox_middle.add(bbox_middle_right);
	outer_box.add(bbox_middle);

	outer_box.add(sep_bottom);

	cmd_btn.set_tooltip_text("Publish a command to the vehicle");
	calibrate.set_tooltip_text("Calibrate cameras on vehicle");

	outer_box.add(xp), outer_box.add(yp), outer_box.add(zp), outer_box.add(yawp);
	outer_box.add(xr), outer_box.add(yr), outer_box.add(zr), outer_box.add(yawr);
	outer_box.add(cameras);
	bbox_upper_bottom.add(cmd_btn);
	bbox_upper_bottom.add(calibrate);
	bbox_upper_bottom.add(vision_btn);
	bbox_mid_bottom.add(analog_ctrl_sw);
	bbox_lower_bottom.add(lx);
	bbox_lower_bottom.add(ly);
	bbox_lower_bottom.add(rx);
	bbox_lower_bottom.add(ry);
	bbox_bottom.add(bbox_upper_bottom);
	bbox_bottom.add(bbox_mid_bottom);
	bbox_bottom.add(bbox_lower_bottom);
	outer_box.add(bbox_bottom);

	update_runstate_btn(runstate.stat());
	idle_btn.set_label(u8"\u3000\uff29\uff24\uff2c\uff25\u3000");

	// pressing the mission start/stop button toggles the mission-running state
	mission_btn.signal_clicked().connect([this]() {
		nav_runstate_t runst;
		runst.running_mission = not runstate.stat().running_mission;
		runstate.cmd(runst);
		if (runst.running_mission)
		{
			MAAV_DEBUG("Telling vehicle to run autonomous mission!");
			autonomous.set_active();
			autonomous.set_sensitive(true);
			set_pose.set_sensitive(false);
			takeoff.set_sensitive(false);
			land.set_sensitive(false);
			hover.set_sensitive(false);
			exit_arena.set_sensitive(false);
			cmd_btn.set_sensitive(false);
		}
		else
		{
			MAAV_DEBUG("Telling vehicle to stop mission and hover.");
			hover.set_active();
			autonomous.set_sensitive(false);
			set_pose.set_sensitive(true);
			takeoff.set_sensitive(true);
			land.set_sensitive(true);
			hover.set_sensitive(true);
			exit_arena.set_sensitive(true);
			cmd_btn.set_sensitive(true);
		}
	});

	// Publishes the current waypoint command. This command is modified by the
	// radio buttons and the text fields in the Command Frame.
	cmd_btn.signal_clicked().connect([this]() {
		if (not autonomous.get_active())
		{
			update_waypt();
			waypt.publish_new();
		}
	});

	calibrate.signal_clicked().connect([this]() {
		camera_disc_t cam;
		cam.numCameras = cameras.get_value();
		MAAV_DEBUG("Sending calibrate message with %d cameras", cameras.get_value());
		vision.cmd(cam);
	});

	vision_btn.signal_clicked().connect([this]() {
		nav_runstate_t msg;
		msg.running_mission = true;
		vision_start.cmd(msg);
		vision_btn.set_sensitive(false);
	});

	idle_btn.signal_clicked().connect([this]() {
		idle_t msg;
		msg.idle = true;
		msg.utime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
		idle.cmd(msg);
		// should not send idle message twice
		idle_btn.set_sensitive(false);
		// cannot move quad without idling first
		mission_btn.set_sensitive(true);
		cmd_btn.set_sensitive(true);
	});

	// updates to either StatCmd should update their corresponding buttons
	runstate.signal_update().connect(
		[this](const nav_runstate_t& stat) { update_runstate_btn(stat); });
	waypt.signal_update().connect([this](const waypoint_t& stat) { update_waypt_btns(stat); });

	chk_pose.signal_clicked().connect([this]() {
		if (chk_pose.get_active())
		{
			xp.set_sensitive(true);
			yp.set_sensitive(true);
			zp.set_sensitive(true);
			yawp.set_sensitive(true);
		}
		else
		{
			xp.set_sensitive(false);
			yp.set_sensitive(false);
			zp.set_sensitive(false);
			yawp.set_sensitive(false);
		}
	});

	chk_rate.signal_clicked().connect([this]() {
		if (chk_rate.get_active())
		{
			xr.set_sensitive(true);
			yr.set_sensitive(true);
			zr.set_sensitive(true);
			yawr.set_sensitive(true);
		}
		else
		{
			xr.set_sensitive(false);
			yr.set_sensitive(false);
			zr.set_sensitive(false);
			yawr.set_sensitive(false);
		}
	});

	analog_ctrl_sw.get_switch().property_active().signal_changed().connect([this]() {
		analog_ctrl.prime();
		if (analog_ctrl_sw.get_switch().get_active())
		{
			MAAV_DEBUG("Analog Conroller Input ON");
			analog_ctrl_connection = Glib::signal_timeout().connect(
				[this]() {
					xcontroller.updateControllerState();
					analog_ctrl.cmd(xcontroller.getDesiredRpyt());
					stringstream ss;
					string val;
					auto raw_vals = xcontroller.getRawStickVals();
					ss << fixed << setprecision(2) << raw_vals[2] << " " << raw_vals[3] << " "
					   << raw_vals[0] << " " << raw_vals[1];
					ss >> val;
					lx.set_markup("<tt>LX: " + val + "</tt>");
					ss >> val;
					ly.set_markup("<tt>LY: " + val + "</tt>");
					ss >> val;
					rx.set_markup("<tt>RX: " + val + "</tt>");
					ss >> val;
					ry.set_markup("<tt>RY: " + val + "</tt>");
					return true;
				},
				15  // emit every 15 milliseconds (67Hz)
				);
		}
		else
		{
			MAAV_DEBUG("Analog Conroller Input OFF");
			analog_ctrl_connection.disconnect();
			lx.set_markup("<tt>LX: 0.00</tt>");
			ly.set_markup("<tt>LY: 0.00</tt>");
			rx.set_markup("<tt>RX: 0.00</tt>");
			ry.set_markup("<tt>RY: 0.00</tt>");
		}
	});

	lx.set_use_markup();
	rx.set_use_markup();
	ly.set_use_markup();
	ry.set_use_markup();

	chk_pose.set_active(true);
	chk_rate.set_active(true);

	add(outer_box);
	show_all_children();
}

void CommandFrame::update_waypt()
{
	if (autonomous.get_active())
	{
		waypoint_t way;
		way.pmode = 0;
		way.mode = 0;
		fill(begin(way.pose), end(way.pose), 0.);
		waypt.update(way);
		MAAV_DEBUG("Autonomous command is being sent! (You should never see this)");
		return;
	}
	else if (hover.get_active())
	{
		waypoint_t way;
		way.pmode = 1;
		way.mode = 0;
		fill(begin(way.pose), end(way.pose), 0.);
		waypt.update(way);
		MAAV_DEBUG("Hover command is being sent!");
	}
	else if (takeoff.get_active())
	{
		waypoint_t way;
		way.pmode = 2;
		way.mode = 0;
		fill(begin(way.pose), end(way.pose), 0.);
		waypt.update(way);
		MAAV_DEBUG("Takeoff command is being sent!");
	}
	else if (land.get_active())
	{
		waypoint_t way;
		way.pmode = 3;
		way.mode = 0;
		fill(begin(way.pose), end(way.pose), 0.);
		waypt.update(way);
		MAAV_DEBUG("Land command is being sent!");
	}
	else if (set_pose.get_active())
	{
		waypoint_t way;
		way.pmode = 4;
		if (chk_pose.get_active() and chk_rate.get_active())
		{
			way.mode = 3;
			double poses[] = {xp.get_value(), yp.get_value(), -zp.get_value(), yawp.get_value()};
			double rates[] = {xr.get_value(), yr.get_value(), zr.get_value(), yawr.get_value()};
			copy(begin(poses), end(poses), begin(way.pose));
			copy(begin(rates), end(rates), begin(way.rate));
		}
		else if (chk_pose.get_active())
		{
			way.mode = 1;
			double poses[] = {xp.get_value(), yp.get_value(), -zp.get_value(), yawp.get_value()};
			double rates[] = {0., 0., 0., 0.};
			copy(begin(poses), end(poses), begin(way.pose));
			copy(begin(rates), end(rates), begin(way.rate));
		}
		else
		{
			way.mode = 2;
			double poses[] = {0., 0., 0., 0.};
			double rates[] = {xr.get_value(), yr.get_value(), zr.get_value(), yawr.get_value()};
			copy(begin(poses), end(poses), begin(way.pose));
			copy(begin(rates), end(rates), begin(way.rate));
		}
		waypt.update(way);
		MAAV_DEBUG("Pose command is being sent!");
	}
	else
	{
		waypoint_t way;
		way.pmode = 5;
		way.mode = 0;
		fill(begin(way.pose), end(way.pose), 0.);
		waypt.update(way);
		MAAV_DEBUG("Exit arena command is being sent!");
	}
}

void CommandFrame::update_runstate_btn(const nav_runstate_t& stat)
{
	// the start/stop mission button
	// F U L L W I D T H for emphasis
	if (stat.running_mission)
	{
		mission_btn.set_label(
			u8"\u3000\uff33\uff34\uff2f\uff30\u3000"
			u8"\uff2d\uff29\uff33\uff33\uff29\uff2f\uff2e");
	}
	else
	{
		mission_btn.set_label(
			u8"\uff33\uff34\uff21\uff32\uff34\u3000"
			u8"\uff2d\uff29\uff33\uff33\uff29\uff2f\uff2e");
	}
}

void CommandFrame::update_waypt_btns(const waypoint_t& stat)
{
	switch (stat.mode)
	{
		case 0:
			// takeoff.set_sensitive(true);
			// land.set_sensitive(false);
			// hover.set_sensitive(false);
			break;
		case 1:
			// takeoff.set_sensitive(false);
			// land.set_sensitive(true);
			// hover.set_sensitive(false);
			break;
		case 2:
		case 3:
			// takeoff.set_sensitive(false);
			// land.set_sensitive(true);
			// hover.set_sensitive(true);
			break;
	}
}
}
}
