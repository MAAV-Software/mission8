#ifndef GETOPT_HPP
#define GETOPT_HPP

#include "getopt.h"
#include "zarray.h"

/**
 * @brief Command line arugment parser (getopt)
 * @details C++ wrapper over April Lab's (april.eecs.umich.edu) C-style getopt
 *			implementation used in EECS 467 and ROB 550 at the University of
 *			Michigan.
 *
 *			An example use case of this is as follows:
 *			////////////////////////////////////////////////////////////////////////////////////////
 *			#include <iostream>
 *			#include "GetOpt.h"
 *
 *			using std::cout;
 *			using std::endl;
 *
 *			int main(int argc, char** argv)
 *			{
 *				GetOpt gopt;
 *				gopt.addString('d', "device", "/dev/ttyO1", "SAMA5 comms device");
 *				gopt.addInt('b', "baud", "230400", "Baud rate");
 *
 *				// the following option does not have a short-form
 *				gopt.addInt('\0', "sync-interval", "499", "Interval between sync requests (ms)");
 *
 *				// the "help" option is always available by default
 *   			if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
 *				{
 *       			cout << "Usage: " << argv[0] << " [options]" <<  endl;
 *					gopt.printHelp();
 *       			return 1;
 *   			}
 *
 *				string device{gopt.getString("device")};
 *				int baud{gopt.getInt("baud"), sync{gopt.getInt("sync-interval")};
 *				// Use device, baud, and sync as variables
 *
 *				return 0;
 *			}
 *			//////////////////////////////////////////////////////////////////////////////////////////
 *
 *			In the terminal type (assuming the program name is "pgrm"):
 *			./pgrm -d /dev/tty01 -b 115200 --syn-interval 15
 *
 * @author Sajan Patel (sajanptl)
 */
class GetOpt
{
public:
    /**
     * @brief Contructs a GetOpt object
     * @details Underlying implementation allocates memory on the heap.
     */
    GetOpt();

    /**
     * @brief Destroys a GetOpt object and deallocates underlying memory.
     */
    ~GetOpt();

    /**
     * @brief Disabling copy constructor and copy assignment.
     * @details The underlying getopt_t pointer from the C library is opaque and
     *			thus the sizes are not available along with other members that
     *			would need to be copied (perhaps other pointers). This leads the
     *			author to the decision that since the underlying C implementation
     *			did not supply the user with a copy support, this class wrapper
     *			will not support the copy constructor or copy assignment.
     *			However, the author will support move functionality as it does
     *			not depend on copying or knowing the underlying structor of the
     *			data; it just moves it (helpful for use with C++ 11 STL thread
     *			library).
     */
    GetOpt(const GetOpt &original) = delete;
    GetOpt &operator=(const GetOpt &rhs) = delete;

    /**
     *@brief GetOpt move constructor
     */
    GetOpt(GetOpt &&original);

    /**
     * @brief GetOpt move assignment
     */
    GetOpt &operator=(GetOpt &&rhs);

    /**
     * @brief Parses the command line arguments and returns true upon success.
     * @details Given argc and argv directly from the arguments of main, this
     *			function parses the command line arguments into their respective
     *			fields within GetOpt's internal representation. Returns true if
     *			the parsing was successful.
     * @param	argc	number of command line arguments
     * @param	argv	pointer to C-style command line strings (char **)
     * @param	showErrors	if true, prints errors to stdout; does not if false
     * @pre		GetOpt options must be added using the "adder" functions
     * @post	GetOpt options are available through the "getter" functions
     */
    bool parse(int argc, char *argv[], bool showErrors);

    /**
     * @brief Prints the help strings to stdout.
     */
    void printHelp();

    /**
     * @brief Adds a spacer to the command line arguments
     * @params	s 	Spacer character
     */
    void addSpacer(const char *s);

    /**
     * @brief Adds a boolean argument that uses the strings "true" or "false"
     * @details	Adds a boolean option to the command line argument. When using
     *			this argument on the command line, the value passed in is the
     *			string "true" or "false" for true or false values, respectively.
     * @param sopt	Short-form character
     * @param lname	Long-form argument string
     * @param def	Default value
     * @param help	Help text for this option
     */
    void addBool(char sopt, const char *lname, bool def, const char *help);

    /**
     * @brief Adds an integer argument to the command line options
     * @details	The integer must be specified as a string (e.g. "1").
     * @param sopt	Short-form character
     * @param lname	Long-form argument string
     * @param def	Default value
     * @param help	Help text for this option
     */
    void addInt(char sopt, const char *lname, const char *def, const char *help);

    /**
     * @brief Adds an string argument to the command line options
     * @param sopt	Short-form character
     * @param lname	Long-form argument string
     * @param def	Default value
     * @param help	Help text for this option
     */
    void addString(char sopt, const char *lname, const char *def, const char *help);

    /**
     * @brief Adds a double value argument to the command line options
     * @details	The double value must be specified as a string (e.g. "1.01").
     * @param sopt	Short-form character
     * @param lname	Long-form argument string
     * @param def	Default value
     * @param help	Help text for this option
     */
    void addDouble(char sopt, const char *lname, const char *def, const char *help);

    /**
     * @brief Returns the string for the given option
     * @param lname	Long-form name of the option
     */
    const char *getString(const char *lname);

    /**
     * @brief Returns the integer value for the given option
     * @param lname	Long-form name of the option
     */
    int getInt(const char *lname);

    /**
     * @brief Returns the boolean value for the given option
     * @param lname	Long-form name of the option
     */
    bool getBool(const char *lname);

    /**
     * @brief Returns the double value for the given option
     * @param lname	Long-form name of the option
     */
    double getDouble(const char *lname);

    /**
     * @brief Returns true if the option/argument was specified
     * @param lname	Long-form name of the option
     */
    bool wasSpecified(const char *lname);

    /**
     * @brief Returns the extra command line arguments
     * @details	The author (for time constrainsts and practical reasons) is
     *			simply wrapping around the underlying C function to return the
     *			extra arguments in the form of a zarray_t. This is just for
     *			posterity, and the author will not put in the effort to convert
     *			the form of the response into a vector or some other C++ STL
     *			container. The user can do the work if he wishes; however,
     *	 		if proper command line argument style is used, the user should
     *			not need this function at all.
     */
    const zarray_t *getExtraArgs();

private:
    getopt_t *gopt;  ///< underlying getopt pointer

    /**
     * @brief Swap wrapper around std::swap to make Move Ctor/Assignment easy
     */
    void swap(GetOpt &other) noexcept;
};

#endif /* GetOpt.hpp */
