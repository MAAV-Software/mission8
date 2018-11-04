#include "common/utils/GetOpt.hpp"
#include "common/utils/getopt.h"
#include "common/utils/zarray.h"

#include <utility>

GetOpt::GetOpt() : gopt(getopt_create()) {}
GetOpt::~GetOpt() { getopt_destroy(gopt); }
void GetOpt::swap(GetOpt &other) noexcept { std::swap(gopt, other.gopt); }
GetOpt::GetOpt(GetOpt &&original) : GetOpt()
{
    swap(original);  // move ctor is just construct and swap from original
}

GetOpt &GetOpt::operator=(GetOpt &&rhs)
{
    swap(rhs);  // use swap idiom to move the data
    return *this;
}

bool GetOpt::parse(int argc, char *argv[], bool showErrors)
{
    return getopt_parse(gopt, argc, argv, showErrors ? 1 : 0) == 1;
}

void GetOpt::printHelp() { getopt_do_usage(gopt); }
void GetOpt::addSpacer(const char *s) { getopt_add_spacer(gopt, s); }
void GetOpt::addBool(char sopt, const char *lname, bool def, const char *help)
{
    getopt_add_bool(gopt, sopt, lname, def ? 1 : 0, help);
}

void GetOpt::addInt(char sopt, const char *lname, const char *def, const char *help)
{
    getopt_add_int(gopt, sopt, lname, def, help);
}

void GetOpt::addString(char sopt, const char *lname, const char *def, const char *help)
{
    getopt_add_string(gopt, sopt, lname, def, help);
}

void GetOpt::addDouble(char sopt, const char *lname, const char *def, const char *help)
{
    getopt_add_double(gopt, sopt, lname, def, help);
}

const char *GetOpt::getString(const char *lname) { return getopt_get_string(gopt, lname); }
int GetOpt::getInt(const char *lname) { return getopt_get_int(gopt, lname); }
bool GetOpt::getBool(const char *lname) { return getopt_get_bool(gopt, lname) == 1; }
double GetOpt::getDouble(const char *lname) { return getopt_get_double(gopt, lname); }
bool GetOpt::wasSpecified(const char *lname) { return getopt_was_specified(gopt, lname) == 1; }
const zarray_t *GetOpt::getExtraArgs() { return getopt_get_extra_args(gopt); }
