#ifndef TOOLS_H
#define TOOLS_H

#include <vector>
#include <math.h>

class Tools {

  public:

    Tools();

    // For converting back and forth between radians and degrees.
    double pi() { return M_PI; }
    double deg2rad(const double &x) { return x * pi() / 180; }
    double rad2deg(const double &x) { return x * 180 / pi(); }

    // Normalizes an angle to be in the range [-pi, pi]
    double normalizeAngle(const double &x);

    // For doing translations and rotations of cartesian coordinates
    void translate(double &x, double &y, const double &trans_x, const double &trans_y);
    void rotate(double &x, double &y, const double &angle);
    void translateAndRotate(double &x, double &y,
                            const double &trans_x, const double &trans_y, const double &angle);
    void rotateAndTranslate(double &x, double &y,
                            const double &angle, const double &trans_x, const double &trans_y);

    // Converts MPH to m/s
    double mph2ms(const double &x);

    // Calculates the distance between two points
    double distance(const double &x1, const double &y1, const double &x2, const double &y2);

    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    std::vector<double> getFrenet(const double &x, const double &y, const double &theta,
                                  const std::vector<double> &maps_x, const std::vector<double> &maps_y);

    // Transform from Frenet s,d coordinates to Cartesian x,y
    std::vector<double> getXY(const double &s, const double &d,
                              const std::vector<double> &maps_s,
                              const std::vector<double> &maps_x, const std::vector<double> &maps_y);

  private:

    int ClosestWaypoint(const double &x, const double &y,
                        const std::vector<double> &maps_x, const std::vector<double> &maps_y);

    int NextWaypoint(const double &x, const double &y, const double &theta,
                     const std::vector<double> &maps_x, const std::vector<double> &maps_y);
};

#endif /* TOOLS_H */
