#ifndef MAAV_GCS_GCS_HPP
#define MAAV_GCS_GCS_HPP

#include "CommandFrame.hpp"
#include "StatusFrame.hpp"
#include "TuningFrame.hpp"

#include <string>

#include <gtkmm/box.h>
#include <gtkmm/window.h>

namespace maav
{
/**
 * @brief The Ground Control Station and related UI utilities
 */
namespace gcs
{
/**
 * @brief The top-level GCS window
 */
class GCS : public Gtk::Window
{
	// the ZCM connection for this window
	GlibZCM zcm;
	// Constants container
	const GCSConsts& CONSTS;

	// the status, command, and tuning frames displayed by this window
	StatusFrame status{zcm, CONSTS};
	CommandFrame command{zcm, CONSTS};
	TuningFrame tuning{zcm, CONSTS};

	// the boxes that all of those go into
	Gtk::Box outer_box{Gtk::ORIENTATION_VERTICAL}, inner_box;

   public:
	/**
	 * @brief Sets up a Ground Control Station window for display
	 */
	GCS(const std::string& zcm_url, const GCSConsts& consts_in);
};
}
}

#endif
