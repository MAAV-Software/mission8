/*
 * RingBuffer.hpp
 *
 *  Created on: May 20, 2015
 *      Author: Sasawat
 */

#ifndef RINGBUFFER_HPP_
#define RINGBUFFER_HPP_

#include <stdint.h>

//N MUST BE A POWER OF TWO 2 4 8 16 YOU KNOW THE DEAL
template <uint32_t N>
class RingBuffer
{
public:
    // Constructs RingBuffer with allocated memory of N bytes
    RingBuffer() : writer(0), reader(0), mask(N - 1) { data = new uint8_t[N]; }
    ~RingBuffer() { delete[] data; }

    //REQUIRES: If RINGBUF_NO_OVERWRITE is not defined, RingBuffer should have
    //          space remaining (check using the unwritten call) otherwise
    //          data may be lost.
    //EFFECTS:  Puts datum into the RingBuffer. If RINGBUF_NO_OVERWRITE is
    //          defined, returns false if no space is remaining. If it is not
    //          defined, datum will always be pushed into the RingBuffer,
    //          pushing the write head beyond the read head, rendering ALL DATA
    //          PREVIOUSLY IN THE RINGBUFFER INACCESSIBLE.
    bool push(uint8_t datum);

    //REQUIRES: The RingBuffer to have bytes left to read (check using unread).
    //EFFECTS:  If possible, returns the last unread byte in the RingBuffer,
    //          otherwise returns 0. Note that 0 can be a valid value in the
    //          RingBuffer so checks against 0 to defend against over-reading
    //          RingBuffer will not work (unless you are guaranteed that 0 is
    //          not a valid value).
    uint8_t pop();

    //EFFECTS:  Returns the number of unread bytes in the RingBuffer
    uint32_t unread() const;

    //EFFECTS:  Returns the number of bytes left to be written in the RingBuffer
    uint32_t unwritten() const;

    //EFFECTS:  Clears the RingBuffer
    void clear();
	
private:
    uint8_t* data;
    uint32_t writer;
    uint32_t reader;
    const uint32_t mask;
};

template <uint32_t N>
bool RingBuffer<N>::push(uint8_t datum)
{
    // Enabling or disabling overwrite on push. If overwrite on push is enabled
    // things will be faster since pushing will be like branchless
    // But you A LOT of data as the write header will get in front of the read header
    // When that happens all unread data is lost
    // In messaging we will read very often from the buffer, so this isn't an issue
    // So I left it off since AFAIK, that is the only use of the RingBuffer so far
#ifdef RINGBUF_NO_OVERWRITE
    if(!ringbuf_spaceLeft(ringbuffer))
    {
        return false;
    }
#endif

    // Push the byte in
    data[writer] = datum;

    // Advance the write head
    ++writer;

    // Warparound the write head
    writer = writer & mask;

    // Success!
    return true;
}

template <uint32_t N>
uint8_t RingBuffer<N>::pop()
{
    // We literally can't defend against the user trying to read more data than availabe
    // Since all returns values are valid (Yes we can use Exceptions, but possible performance hit?)
    // So all we can do is guard against incrementing the reader past the reader
    if(reader == writer)
    	return 0;

    // Read the data
    uint8_t ret = data[reader];

    // Advance the read head
    ++reader;

    // Warp around the read head
    reader = reader & mask;

    // Return the read data
    return ret;
}

template <uint32_t N>
uint32_t RingBuffer<N>::unread() const
{
    return (writer - reader + mask + 1) % (mask + 1);
}

template <uint32_t N>
uint32_t RingBuffer<N>::unwritten() const
{
    return (reader - writer - 1 + mask + 1) % (mask + 1);
}

template <uint32_t N>
void RingBuffer<N>::clear()
{
    writer = reader = 0;
}


#endif /* RINGBUFFER_HPP_ */
