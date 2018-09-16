#ifndef RECORD_HPP
#define RECORD_HPP

#include <memory>
#include <cstdint>
#include <utility>

/**
 * @brief Templates and aliasing to create Record and RecordPtr for data
 *		  associated with a timestamp (64-bit integer, e.g. in microseconds).
 */
template <typename T>
using Record = std::pair<T, int64_t>;
template <typename T>
using RecordPtr = std::shared_ptr<Record<T>>;

#endif
