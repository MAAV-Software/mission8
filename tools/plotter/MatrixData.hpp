#include <vector>
#include "AbstractData.hpp"

class MatrixData : public AbstractData
{
public:
    virtual void addData(std::vector<double>&& data) override;
};