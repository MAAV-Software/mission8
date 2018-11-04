#include "common/math/FieldOfView.hpp"
#include <cmath>

namespace maav
{
/*
I will explain here what h, l, w, d stands for in the following two functions.

Consider an image taken by a camera:

           E
  A  -------------  D
     |     |     |
  F  |-----------|  H
     |     | O   |
  B  -------------  C
           G

Then also consider the three dimensions: horizontal, vertical, and diagonal:

  Horizontal                Vertical                 Diagonal

       /| F                     /| E                     /| D
      / |                      / |                      / |
     /  |                     /  |                     /  |
  P <---| O                P <---| O                P <---| O
     \  |                     \  |                     \  |
      \ |                      \ |                      \ |
       \| H                     \| G                     \| B

P is where the camera is at. O is the center of the image.
d is the length of OD.
w is the length of OF.
h is the length of OE.
l is the length of OP.
*/

FieldOfView::FieldOfView(double hFOV, double vFOV) : horizontalFOV(hFOV), verticalFOV(vFOV)
{
    double h = 1;
    double l = h / tan(vFOV / 2);
    double w = l * tan(hFOV / 2);
    double d = sqrt(h * h + w * w);
    diagonalFOV = 2 * atan(d / l);
}

FieldOfView::FieldOfView(int width, int height, double dFOV) : diagonalFOV(dFOV)
{
    double h = height;
    double w = width;
    double d = sqrt(h * h + w * w);
    double l = d / tan(dFOV / 2);
    horizontalFOV = 2 * atan(w / l);
    verticalFOV = 2 * atan(h / l);
}

double FieldOfView::hFOV() const { return horizontalFOV; }
double FieldOfView::vFOV() const { return verticalFOV; }
double FieldOfView::dFOV() const { return diagonalFOV; }
}  // namespace maav
