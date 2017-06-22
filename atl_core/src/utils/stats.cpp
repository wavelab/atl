#include "atl/utils/stats.hpp"


namespace atl {

inline static double sqr(double x) {
  return x * x;
}

int linreg(std::vector<Vec2> pts, double *m, double *c, double *r) {
  // linear regression of form: y = mx + c
  Vec2 p;
  double sumx = 0.0;  /* sum of x */
  double sumx2 = 0.0; /* sum of x^2 */
  double sumxy = 0.0; /* sum of x * y */
  double sumy = 0.0;  /* sum of y */
  double sumy2 = 0.0; /* sum of y^2 */

  for (int i = 0; i < pts.size(); i++) {
    p = pts[i];
    sumx += p(0);
    sumx2 += sqr(p(0));
    sumxy += p(0) * p(1);
    sumy += p(1);
    sumy2 += sqr(p(1));
  }

  double denom = (pts.size() * sumx2 - sqr(sumx));
  if (denom == 0) {
    // singular matrix. can't solve the problem.
    *m = 0;
    *c = 0;
    if (r) {
      *r = 0;
    }
    return -1;
  }

  *m = (pts.size() * sumxy - sumx * sumy) / denom;
  *c = (sumy * sumx2 - sumx * sumxy) / denom;
  /* compute correlation coeff */
  if (r != NULL) {
    *r = (sumxy - sumx * sumy / pts.size());
    *r /= sqrt((sumx2 - sqr(sumx) / pts.size()) *
               (sumy2 - sqr(sumy) / pts.size()));
  }

  return 0;
}

}  // end of atl namespace
