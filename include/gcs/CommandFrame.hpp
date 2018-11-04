#ifndef MAAV_GCS_COMMAND_FRAME_HPP
#define MAAV_GCS_COMMAND_FRAME_HPP

#include "GCSConsts.hpp"
#include "StatCmd.hpp"
#include "XboxController.hpp"
#include "zcm_type_ops.hpp"

#include <gtkmm/box.h>
#include <gtkmm/button.h>
#include <gtkmm/buttonbox.h>
#include <gtkmm/checkbutton.h>
#include <gtkmm/entry.h>
#include <gtkmm/frame.h>
#include <gtkmm/label.h>
#include <gtkmm/radiobutton.h>
#include <gtkmm/scale.h>
#include <gtkmm/separator.h>
#include <gtkmm/switch.h>

#include <iomanip>
#include <sstream>

namespace maav
{
namespace gcs
{
/**
 * @brief The GCS command frame
 *
 * @details This frame contains buttons which can be used to send commands to
 * the vehicle and a set of text entries for specifying parameters for some of
 * those commands
 */
class CommandFrame : public Gtk::Frame
{
   public:
    /**
     * @brief Sets up a command frame
     */
    CommandFrame(GlibZCM&, const GCSConsts& consts_in);

   private:
    // Constants container
    const GCSConsts& CONSTS;

    // a Gtk::Entry which allows only numbers to be entered.
    class NumberEntry : public Gtk::Entry
    {
       public:
        /**
        * @brief Sets up a Gtk::Entry that takes in inly numbers
        *
        * @param allow_floats_in Whether to allow floating point numbers
        */
        explicit NumberEntry(bool allow_floats_in);

        // Override of Gtk::Entry to ensure only numbers are inserted in the entry
        void on_insert_text(const Glib::ustring& text, int* position) override;

       private:
        // Checks if every character in text is a numeral
        // allows a single dot ('.') for floating point numbers
        bool is_numeral(const Glib::ustring& text);

        // Whether this Number Entry allows floats to be entered
        bool allow_floats;
    };

    // a helper widget which shows an entry paired with a given label for
    // entering numbers
    template <typename T>
    class LabeledNumEntry : public Gtk::Box
    {
        Gtk::Label label;
        NumberEntry entry;
        const GCSConsts& CONSTS;

       public:
        // constructs a labeled entry; name is used for the label text
        // and entr is used as the default display on the entry.
        explicit LabeledNumEntry(const GCSConsts& consts_in, const char* name,
            const char* entr = "0.0", bool allow_floats = true)
            : label{std::string{"<tt>"} + name + "</tt>"}, entry{allow_floats}, CONSTS{consts_in}
        {
            set_spacing(CONSTS.SMALL_SPACE);

            label.set_use_markup();

            entry.set_placeholder_text(entr);

            add(label);
            pack_start(entry);
        }

        // retrieves the entry's value
        T get_value()
        {
            std::stringstream ss;
            // if entry is empty, return 0
            if (entry.get_text() == "")
            {
                ss << "0";
            }
            else
            {
                ss << entry.get_text();
            }
            T val;
            ss >> val;
            return val;
        }

        Glib::ustring get_text() { return entry.get_text(); }
    };

    class LabeledSwitch : public Gtk::Box
    {
       public:
        explicit LabeledSwitch(const GCSConsts& consts_in, const char* name)
            : label{name}, CONSTS{consts_in}
        {
            set_spacing(CONSTS.SMALL_SPACE);
            label.set_use_markup();
            swt.set_active(false);
            add(label);
            pack_start(swt);
        }

        bool get_active() { return swt.get_active(); }
        Gtk::Switch& get_switch() { return swt; }
       private:
        Gtk::Label label;
        Gtk::Switch swt;
        const GCSConsts& CONSTS;
    };

    // the top-level box of the frame
    Gtk::Box outer_box{Gtk::ORIENTATION_VERTICAL};

    // the button box at the top and its one button
    Gtk::ButtonBox bbox_top{Gtk::ORIENTATION_HORIZONTAL};
    Gtk::Button mission_btn;
    Gtk::Button idle_btn;

    // the separators between the sections
    Gtk::Separator sep_top, sep_bottom;

    // the middle button box
    Gtk::ButtonBox bbox_middle{Gtk::ORIENTATION_HORIZONTAL};
    Gtk::ButtonBox bbox_middle_left{Gtk::ORIENTATION_VERTICAL};
    Gtk::ButtonBox bbox_middle_right{Gtk::ORIENTATION_VERTICAL};
    Gtk::RadioButton autonomous{"Autonomous"}, set_pose{"Pose"}, takeoff{"Takeoff"}, land{"Land"},
        hover{"Hover"}, exit_arena{"Exit Arena"};
    Gtk::CheckButton chk_pose{"Pose"}, chk_rate{"Rate"};
    LabeledSwitch analog_ctrl_sw{CONSTS, "Analog Game Controller"};

    // the labeled text entries and buttons which make up the bottom part of the
    // frame
    LabeledNumEntry<double> xp{CONSTS, "X Pose:     "}, yp{CONSTS, "Y Pose:     "},
        zp{CONSTS, "Z Pose:     "}, yawp{CONSTS, "Yaw Pose:   "};
    LabeledNumEntry<double> xr{CONSTS, "X Rate:     "}, yr{CONSTS, "Y Rate:     "},
        zr{CONSTS, "Z Rate:     "}, yawr{CONSTS, "Yaw Rate:   "};
    LabeledNumEntry<int> cameras{CONSTS, "Cameras:    ", "0", false};
    Gtk::ButtonBox bbox_bottom{Gtk::ORIENTATION_VERTICAL};
    Gtk::ButtonBox bbox_upper_bottom{Gtk::ORIENTATION_HORIZONTAL};
    Gtk::ButtonBox bbox_mid_bottom{Gtk::ORIENTATION_HORIZONTAL};
    Gtk::ButtonBox bbox_lower_bottom{Gtk::ORIENTATION_HORIZONTAL};
    Gtk::Button cmd_btn{"Publish Command"}, calibrate{"Calibrate Cameras"};
    ;
    Gtk::Button vision_btn{"Start Vision"};

    // StatCmds for sending commands to the controller and path planner
    StatCmd<nav_runstate_t> runstate;
    StatCmd<nav_runstate_t> vision_start;
    StatCmd<waypoint_t> waypt;
    StatCmd<camera_disc_t> vision;
    StatCmd<idle_t> idle;
    StatCmd<dji_t> analog_ctrl;

    // Label to show raw stick values for analog controller
    Gtk::Label lx{"<tt>LX: 0.00</tt>"}, ly{"<tt>LY: 0.00</tt>"}, rx{"<tt>RX: 0.00</tt>"},
        ry{"<tt>RY: 0.00</tt>"};

    // Connection used to send xbox controller input to vehicle.
    sigc::connection analog_ctrl_connection;

    // XboxController used to send input to the vehicle
    XboxController xcontroller;

    // functions for updating buttons when status updates come from either the path
    // planner or the controller
    void update_runstate_btn(const nav_runstate_t&);
    void update_waypt_btns(const waypoint_t&);

    // Updates waypt with what is currently entered and selected on this Command Frame
    void update_waypt();
};
}
}

#endif
