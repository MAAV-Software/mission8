#ifndef MAAV_MSG_UTIL_H
#define MAAV_MSG_UTIL_H

#include <vector>
#include "gains_t.h"
#include "setpt_t.h"

namespace maav
{
/**
 * @brief Messaging utility functions
 */

/**
 * @brief Transform a setpt_t to a vector<unsigned char>
 * @param pt Setpoint pointer
 * @return vector<unsigned char>
 */
inline std::vector<unsigned char> to_bytes(const setpt_t *pt)
{
	size_t len = setpt_t_encoded_size(nullptr);
	std::vector<unsigned char> ret;

	ret.resize(len);

	setpt_t_encode(ret.data(), 0, len, pt);

	return ret;
}

/**
 * @brief Transform a gains_t to a vector<unsigned char>
 * @param g gains pointer
 * @return vector<unsigned char>
 */
inline std::vector<unsigned char> to_bytes(const gains_t *g)
{
	size_t len = gains_t_encoded_size(nullptr);
	std::vector<unsigned char> ret;

	ret.resize(len);

	gains_t_encode(ret.data(), 0, len, g);

	return ret;
}

} /* namespace maav */

#endif /* MAAV_MSG_UTIL_H */
