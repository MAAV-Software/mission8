#ifndef MSGVALIDATOR_HPP
#define MSGVALIDATOR_HPP

#include <cstdint>

/**
 * @brief Message validator based on monotonically increasing timestamps
 * @author Sajan Patel (sajanptl)
 */
class MsgValidator
{
    public:
    /**
     * @brief Contructs a MsgValidator and sets the internal prevTime to 0
     */
    MsgValidator();

    /**
     * @brief Returns true if the given timestamp is valid (greater than prevTime).
     * @param utime	current message time stamp
     */
    bool operator()(int64_t utime);

    private:
    int64_t prevTime;  ///< previous time stamp
};

#endif /* MsgValidator.hpp */
