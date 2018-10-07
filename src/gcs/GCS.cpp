#include "gcs/GCS.hpp"

using namespace std;

namespace maav
{
namespace gcs
{
GCS::GCS(const string& zcm_url, const GCSConsts& consts_in) : zcm{zcm_url}, CONSTS{consts_in}
{
	set_title("MAAV Ground Control Station");
	set_position(Gtk::WIN_POS_CENTER);

	outer_box.set_border_width(CONSTS.MED_SPACE);
	outer_box.set_spacing(CONSTS.MED_SPACE);
	outer_box.add(status);

	inner_box.set_spacing(CONSTS.MED_SPACE);
	inner_box.add(command);
	inner_box.pack_start(tuning);

	outer_box.pack_start(inner_box);
	add(outer_box);

	show_all_children();
}
}
}
