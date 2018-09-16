#ifndef MAAV_FIELD_OF_VIEW_HPP
#define MAAV_FIELD_OF_VIEW_HPP

namespace maav
{

/**
 * @brief Class for handling camera fields of view (FOV).
 * @author Yichen Yao
 * @details The class handles the transformation between the x and y fields
 * of view and the diagonal field of view.
 */
class FieldOfView
{
public:

	/**
	 * @brief Constructor
	 * @param hFOV the horizontal, or x field of view in radians.
	 * @param vFOV the vertical, or y field of view in radians.
	 */
	FieldOfView(double hFOV, double vFOV);

	/**
	 * @brief Constructor
	 * @param xResolution the number of pixels along the width of the image.
	 * @param yResolution the number of pixels along the height of the image.
	 * @param dFOV the diagonal field of view in radians.
	 */
	FieldOfView(int xResolution, int yResolution, double dFOV);

	/**
	 * @return the hortizontal or x field of view in radians.
	 */
	double hFOV() const;

	/**
	 * @return the vertical or y field of view in radians.
	 */
	double vFOV() const;

	/**
	 * @return the diagonal field of view in radians.
	 */
	double dFOV() const;

private:

	double horizontalFOV;
	double verticalFOV;
	double diagonalFOV;
};

}	// namespace maav

#endif // MAAV_FIELD_OF_VIEW_HPP
