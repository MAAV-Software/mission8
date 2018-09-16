#include "gcs/StatusFrame.hpp"

#include <cassert>
#include <cstdio>

using namespace std;

namespace maav
{
namespace gcs
{

const string StatusFrame::status_text(Status stat, const char* extra)
{
	switch (stat) {
	case Status::online:
		return u8"<span foreground=\"green\"><tt>\U0001f60e ONLINE</tt></span>";
	case Status::quiet:
		return
			u8"<span foreground=\"yellow\"><tt>\U0001f610 QUIET </tt></span>";
	case Status::down:
		return u8"<span foreground=\"red\"><tt>\U0001f615 DOWN  </tt></span>";
	case Status::calibrated:
		char text[256];
		sprintf(text,
			u8"<span foreground=\"green\"><tt>\U0001f60e CALIBRATED %s</tt></span>",
			extra);
		return string{text};
	case Status::uncalibrated:
		return u8"<span foreground=\"red\"><tt>\U0001f615 UNCALIBRATED</tt></span>";
	}
	assert(!"Bad status value!");
}

StatusFrame::StatusFrame(GlibZCM& zcm, const GCSConsts& consts_in) : Gtk::Frame("Status"),
	CONSTS{consts_in},
	path_planner{zcm, NAV_RUNSTATE_STAT, "Mission Planner:", CONSTS},
	localization{zcm, STATE_CHANNEL, "Localization:", CONSTS},
	line_det{zcm, VISION_STAT, "Line Det.:", CONSTS},
	roomba_det{zcm, VISION_STAT, "Roomba Det.:", CONSTS},
	obstacle_det{zcm, OBST_HEARTBEAT_CHANNEL, "Obstacle Det.:", CONSTS},
	controller{zcm, CTRL_HEARTBEAT_CHANNEL, "Controller:", CONSTS},
	camera_cal{zcm, CAMERA_DISC_STAT, CONSTS},
	planner_info{zcm, PLANNER_STAT, CONSTS},
	controller_info{zcm, CTRL_HEARTBEAT_CHANNEL, CONSTS}
{
	MAAV_DEBUG("Initializing Status Frame");
	set_label_align(Gtk::ALIGN_CENTER);

	outer_box.set_border_width(CONSTS.MED_SPACE);
	outer_box.set_spacing(CONSTS.MED_SPACE);
	upper_box.set_spacing(CONSTS.LARGE_SPACE);
	lower_box.set_spacing(CONSTS.LARGE_SPACE);
	under_box.set_spacing(CONSTS.LARGE_SPACE);

	upper_box.add(path_planner), upper_box.add(localization),
		upper_box.add(controller);
	upper_cbox.set_center_widget(upper_box);
	outer_box.add(upper_cbox);

	lower_box.add(line_det), lower_box.add(roomba_det),
		lower_box.add(obstacle_det);
	lower_cbox.set_center_widget(lower_box);
	outer_box.add(lower_cbox);

	under_box.add(camera_cal);
	under_cbox.set_center_widget(under_box);
	outer_box.add(under_cbox);

	info_box.add(planner_info);
	info_box.add(controller_info);
	info_cbox.set_center_widget(info_box);
	outer_box.add(info_cbox);

	add(outer_box);
	show_all_children();
}

}
}
