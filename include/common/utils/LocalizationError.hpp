#ifndef MAAV_NAV_LOCALIZATION_ERROR_HPP
#define MAAV_NAV_LOCALIZATION_ERROR_HPP

#include <exception>
#include <string>

namespace maav
{
namespace err
{
/**
 * @brief This class indicates an attempt to localize has failed
 */
class LocalizationError : std::exception
{
   public:
	explicit LocalizationError(const std::string &problem) : reason{problem} {}
	virtual const char *what() const noexcept override { return reason.c_str(); }
   private:
	std::string reason;
};

} /* namespace err */

} /* namespace maav */

#endif /* MAAV_NAV_LOCALIZATION_ERROR_HPP */
