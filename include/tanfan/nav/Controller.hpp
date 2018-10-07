#ifndef MAAV_CONTROLLER_HPP
#define MAAV_CONTROLLER_HPP

namespace maav
{
class Controller
{
   public:
	virtual void move(double dx, double dy, double dz) = 0;

	virtual void rotate(double dr, double dp, double dy) = 0;

	virtual void takeoff() = 0;

	virtual void land() = 0;
};

}  // namespace maav

#endif  // MAAV_CONTROLLER_HPP
