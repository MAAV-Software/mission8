#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <cstdint>
#include <deque>       // for std::deque
#include <functional>  // for std::make_pair
#include <memory>      // for std::shared_ptr
#include <utility>     // for std::pair

#include "Record.hpp"

/**
 * @brief Templated History Tracker
 */
template <typename T>
class Tracker
{
public:
    /**
     * @brief Constructs a tracker of given size.
     * @param max	Max size.
     */
    Tracker(size_t max);

    Tracker(Tracker& rhs) = delete;

    Tracker& operator=(const Tracker& rhs) = delete;

    /**
     * @brief Adds a new RecordPtr to the history tracker
     * @details	Adds the given RecordPtr to the end of the history, i.e. the
     *			most current time step in the tracker.
     * @param record 	The new record to add.
     */
    void add(RecordPtr<T> record);

    /**
     * @brief Returns a pair of iterators to records before and on/after queryTime
     * @details Searches history for records whose time either matches or is
     *			just before and after queryTime. In either case, the first
     *			element of the returned pair of deque iterators is for the
     *			RecordPtr whose time is just before the queryTime. The second
     *			element is the iterator to the RecordPtr whose time either
     *			matches or is just after the queryTime. Invalid times return
     *			pairs to (end, begin) for history.
     * @param queryTime	Time
     */
    std::pair<typename std::deque<RecordPtr<T>>::iterator,
        typename std::deque<RecordPtr<T>>::iterator>
    get(int64_t queryTime);

    /**
     * @brief Returns a deque end iterator
     */
    typename std::deque<RecordPtr<T>>::iterator end();

    typename std::deque<RecordPtr<T>>::iterator begin();

    /**
     * @brief Inserts record at into history at the given position.
     * @details Wrapped call around deque insert to insert RecordPtr<T>
     */
    typename std::deque<RecordPtr<T>>::iterator insert(
        typename std::deque<RecordPtr<T>>::iterator position, RecordPtr<T> record);

private:
    size_t maxSize;                    ///< max size for tracker
    std::deque<RecordPtr<T>> history;  ///< deque of tracked history record ptrs
};

///////////////////////////// Implementation ///////////////////////////////////
template <typename T>
Tracker<T>::Tracker(size_t max) : maxSize(max)
{
}

// this method requires the user of this class to properly
// declare the pointers to records
template <typename T>
void Tracker<T>::add(RecordPtr<T> record)
{
    history.push_back(record);
    if (history.size() > maxSize) history.pop_front();
}

template <typename T>
typename std::deque<RecordPtr<T>>::iterator Tracker<T>::end()
{
    return history.end();
}

template <typename T>
typename std::deque<RecordPtr<T>>::iterator Tracker<T>::begin()
{
    return history.begin();
}

// inserts element AT the iterator position-
// returns iterator to newly inserted element
template <typename T>
typename std::deque<RecordPtr<T>>::iterator Tracker<T>::insert(
    typename std::deque<RecordPtr<T>>::iterator position, RecordPtr<T> record)
{
    auto ret = history.emplace(position, record);
    if (history.size() > maxSize) history.pop_front();  // maintain max size

    return ret;
}

// returns a pair of records: the before and the after/on of queryTime
template <typename T>
std::pair<typename std::deque<RecordPtr<T>>::iterator, typename std::deque<RecordPtr<T>>::iterator>
Tracker<T>::get(int64_t queryTime)
{
    // linear search from back to pop_front // TODO replace with binary search?
    for (auto It = history.rbegin(); It != history.rend(); ++It)
    {
        if ((*It)->second < queryTime)
            return std::make_pair(It.base() - 1, It.base());  //<before, after/on>
    }

    if ((history.size() >= 1) && ((*history.begin())->second ==
                                     queryTime))  // special case: we need to get the first element
        return std::make_pair(history.end(), history.begin());

    return std::make_pair(history.end(), history.begin());  // other case: search past the beginning
}

#endif
