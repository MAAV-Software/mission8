#ifndef MAAV_GCS_HPP
#define MAAV_GCS_HPP

/**
 * @page gcs The 'maav-gcs' binary
 *
 * The Ground Control Station is used to control the vehicle and view the state
 * of the vehicle.
 *
 * The GCS window is broken up into three panes. The status pane (at the top)
 * shows which vehicle processes are running: processes are displayed as
 * "online" whenever a message is received from them and transtion to "quiet"
 * and then "down" as time passes without messages being received. The command
 * pane (to the left) shows buttons for sending commands to the vehicle: the
 * large button at the top sends signals to the navigation code to start and
 * stop the mission and the buttons and text boxes below it are used to manually
 * send commands to the controls code. The tuning pane (to the right) is used to
 * updating tuning parameters used in the controls code; all of these are
 * specified using auto-rescaling sliders which change their range whenever
 * their thumbs are dragged to either end of the slider.
 *
 * See maav::gcs::StatCmd for a more in-depth explanation of how to implement
 * processes that are able to communicate with this version of GCS.
 */

#endif
