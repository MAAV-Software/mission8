// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include "tanfan/messaging/imu_t.h"
#include <string.h>

static int __imu_t_hash_computed;
static uint64_t __imu_t_hash;

uint64_t __imu_t_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __imu_t_get_hash) return 0;

    __lcm_hash_ptr cp;
    cp.parent = p;
    cp.v = (void *)__imu_t_get_hash;
    (void)cp;

    uint64_t hash =
        (uint64_t)0xb2bb39b7609f2497LL + __float_hash_recursive(&cp) + __float_hash_recursive(&cp) +
        __float_hash_recursive(&cp) + __float_hash_recursive(&cp) + __float_hash_recursive(&cp) +
        __float_hash_recursive(&cp) + __float_hash_recursive(&cp) + __float_hash_recursive(&cp) +
        __float_hash_recursive(&cp) + __float_hash_recursive(&cp) + __float_hash_recursive(&cp) +
        __float_hash_recursive(&cp) + __float_hash_recursive(&cp) + __float_hash_recursive(&cp) +
        __int64_t_hash_recursive(&cp) + __int32_t_hash_recursive(&cp) +
        __float_hash_recursive(&cp) + __float_hash_recursive(&cp) + __float_hash_recursive(&cp) +
        __float_hash_recursive(&cp) + __float_hash_recursive(&cp) + __float_hash_recursive(&cp);

    return (hash << 1) + ((hash >> 63) & 1);
}

int64_t __imu_t_get_hash(void)
{
    if (!__imu_t_hash_computed)
    {
        __imu_t_hash = (int64_t)__imu_t_hash_recursive(NULL);
        __imu_t_hash_computed = 1;
    }

    return __imu_t_hash;
}

int __imu_t_encode_array(void *buf, int offset, int maxlen, const imu_t *p, int elements)
{
    int pos = 0, element;
    int thislen;

    for (element = 0; element < elements; element++)
    {
        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].refYaw), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].gAccX), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].gAccY), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].gAccZ), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].AccX), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].AccY), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].AccZ), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].AngRateX), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].AngRateY), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].AngRateZ), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].MagX), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].MagY), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].MagZ), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, p[element].M, 9);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].time), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].Timer), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].GyroBiasX), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].GyroBiasY), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].GyroBiasZ), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].AccBiasX), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].AccBiasY), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].AccBiasZ), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;
    }
    return pos;
}

int imu_t_encode(void *buf, int offset, int maxlen, const imu_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __imu_t_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0)
        return thislen;
    else
        pos += thislen;

    thislen = __imu_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0)
        return thislen;
    else
        pos += thislen;

    return pos;
}

int __imu_t_encoded_array_size(const imu_t *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++)
    {
        size += __float_encoded_array_size(&(p[element].refYaw), 1);

        size += __float_encoded_array_size(&(p[element].gAccX), 1);

        size += __float_encoded_array_size(&(p[element].gAccY), 1);

        size += __float_encoded_array_size(&(p[element].gAccZ), 1);

        size += __float_encoded_array_size(&(p[element].AccX), 1);

        size += __float_encoded_array_size(&(p[element].AccY), 1);

        size += __float_encoded_array_size(&(p[element].AccZ), 1);

        size += __float_encoded_array_size(&(p[element].AngRateX), 1);

        size += __float_encoded_array_size(&(p[element].AngRateY), 1);

        size += __float_encoded_array_size(&(p[element].AngRateZ), 1);

        size += __float_encoded_array_size(&(p[element].MagX), 1);

        size += __float_encoded_array_size(&(p[element].MagY), 1);

        size += __float_encoded_array_size(&(p[element].MagZ), 1);

        size += __float_encoded_array_size(p[element].M, 9);

        size += __int64_t_encoded_array_size(&(p[element].time), 1);

        size += __int32_t_encoded_array_size(&(p[element].Timer), 1);

        size += __float_encoded_array_size(&(p[element].GyroBiasX), 1);

        size += __float_encoded_array_size(&(p[element].GyroBiasY), 1);

        size += __float_encoded_array_size(&(p[element].GyroBiasZ), 1);

        size += __float_encoded_array_size(&(p[element].AccBiasX), 1);

        size += __float_encoded_array_size(&(p[element].AccBiasY), 1);

        size += __float_encoded_array_size(&(p[element].AccBiasZ), 1);
    }
    return size;
}

int imu_t_encoded_size(const imu_t *p) { return 8 + __imu_t_encoded_array_size(p, 1); }
int __imu_t_decode_array(const void *buf, int offset, int maxlen, imu_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++)
    {
        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].refYaw), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].gAccX), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].gAccY), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].gAccZ), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].AccX), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].AccY), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].AccZ), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].AngRateX), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].AngRateY), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].AngRateZ), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].MagX), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].MagY), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].MagZ), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, p[element].M, 9);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].time), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].Timer), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].GyroBiasX), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].GyroBiasY), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].GyroBiasZ), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].AccBiasX), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].AccBiasY), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].AccBiasZ), 1);
        if (thislen < 0)
            return thislen;
        else
            pos += thislen;
    }
    return pos;
}

int __imu_t_decode_array_cleanup(imu_t *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++)
    {
        __float_decode_array_cleanup(&(p[element].refYaw), 1);

        __float_decode_array_cleanup(&(p[element].gAccX), 1);

        __float_decode_array_cleanup(&(p[element].gAccY), 1);

        __float_decode_array_cleanup(&(p[element].gAccZ), 1);

        __float_decode_array_cleanup(&(p[element].AccX), 1);

        __float_decode_array_cleanup(&(p[element].AccY), 1);

        __float_decode_array_cleanup(&(p[element].AccZ), 1);

        __float_decode_array_cleanup(&(p[element].AngRateX), 1);

        __float_decode_array_cleanup(&(p[element].AngRateY), 1);

        __float_decode_array_cleanup(&(p[element].AngRateZ), 1);

        __float_decode_array_cleanup(&(p[element].MagX), 1);

        __float_decode_array_cleanup(&(p[element].MagY), 1);

        __float_decode_array_cleanup(&(p[element].MagZ), 1);

        __float_decode_array_cleanup(p[element].M, 9);

        __int64_t_decode_array_cleanup(&(p[element].time), 1);

        __int32_t_decode_array_cleanup(&(p[element].Timer), 1);

        __float_decode_array_cleanup(&(p[element].GyroBiasX), 1);

        __float_decode_array_cleanup(&(p[element].GyroBiasY), 1);

        __float_decode_array_cleanup(&(p[element].GyroBiasZ), 1);

        __float_decode_array_cleanup(&(p[element].AccBiasX), 1);

        __float_decode_array_cleanup(&(p[element].AccBiasY), 1);

        __float_decode_array_cleanup(&(p[element].AccBiasZ), 1);
    }
    return 0;
}

int imu_t_decode(const void *buf, int offset, int maxlen, imu_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __imu_t_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0)
        return thislen;
    else
        pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __imu_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0)
        return thislen;
    else
        pos += thislen;

    return pos;
}

int imu_t_decode_cleanup(imu_t *p) { return __imu_t_decode_array_cleanup(p, 1); }
int __imu_t_clone_array(const imu_t *p, imu_t *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++)
    {
        __float_clone_array(&(p[element].refYaw), &(q[element].refYaw), 1);

        __float_clone_array(&(p[element].gAccX), &(q[element].gAccX), 1);

        __float_clone_array(&(p[element].gAccY), &(q[element].gAccY), 1);

        __float_clone_array(&(p[element].gAccZ), &(q[element].gAccZ), 1);

        __float_clone_array(&(p[element].AccX), &(q[element].AccX), 1);

        __float_clone_array(&(p[element].AccY), &(q[element].AccY), 1);

        __float_clone_array(&(p[element].AccZ), &(q[element].AccZ), 1);

        __float_clone_array(&(p[element].AngRateX), &(q[element].AngRateX), 1);

        __float_clone_array(&(p[element].AngRateY), &(q[element].AngRateY), 1);

        __float_clone_array(&(p[element].AngRateZ), &(q[element].AngRateZ), 1);

        __float_clone_array(&(p[element].MagX), &(q[element].MagX), 1);

        __float_clone_array(&(p[element].MagY), &(q[element].MagY), 1);

        __float_clone_array(&(p[element].MagZ), &(q[element].MagZ), 1);

        __float_clone_array(p[element].M, q[element].M, 9);

        __int64_t_clone_array(&(p[element].time), &(q[element].time), 1);

        __int32_t_clone_array(&(p[element].Timer), &(q[element].Timer), 1);

        __float_clone_array(&(p[element].GyroBiasX), &(q[element].GyroBiasX), 1);

        __float_clone_array(&(p[element].GyroBiasY), &(q[element].GyroBiasY), 1);

        __float_clone_array(&(p[element].GyroBiasZ), &(q[element].GyroBiasZ), 1);

        __float_clone_array(&(p[element].AccBiasX), &(q[element].AccBiasX), 1);

        __float_clone_array(&(p[element].AccBiasY), &(q[element].AccBiasY), 1);

        __float_clone_array(&(p[element].AccBiasZ), &(q[element].AccBiasZ), 1);
    }
    return 0;
}

imu_t *imu_t_copy(const imu_t *p)
{
    imu_t *q = (imu_t *)malloc(sizeof(imu_t));
    __imu_t_clone_array(p, q, 1);
    return q;
}

void imu_t_destroy(imu_t *p)
{
    __imu_t_decode_array_cleanup(p, 1);
    free(p);
}
