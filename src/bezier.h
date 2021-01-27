#ifndef BEZIER_H
#define BEZIER_H

#include <iostream>
#include <functional>
#include <cmath>

namespace bezier {

    class point_2d;

    using real_t = double;
    using node_index_t = unsigned int;
    using pcc_t = std::function<point_2d(node_index_t)>;

    namespace constants {
        constexpr int NUM_OF_CUBIC_BEZIER_NODES = 4;
    }

    class point_2d {
    private:
        const real_t _x;
        const real_t _y;
    public:
        point_2d(real_t x, real_t y) : _x(x), _y(y) {}

        friend point_2d operator+(const point_2d&, const point_2d&);

        friend point_2d operator*(const point_2d&, real_t);

        friend point_2d operator*(real_t, const point_2d&);

        friend bool operator==(const point_2d &, const point_2d &);

        friend std::ostream &operator<<(std::ostream &os, const point_2d &a);

        friend pcc_t Scale(pcc_t f, real_t x, real_t y);

        friend pcc_t Rotate(pcc_t f, real_t a);
    };

    point_2d operator+(const point_2d &a, const point_2d &b) {
        return point_2d(a._x + b._x, a._y + b._y);
    }

    point_2d operator*(const point_2d &a, real_t r) {
        return point_2d(a._x * r, a._y * r);
    }

    point_2d operator*(real_t r, const point_2d &a) {
        return a * r;
    }

    bool operator==(const point_2d &a, const point_2d &b) {
        return a._x == b._x && a._y == b._y;
    }

    std::ostream &operator<<(std::ostream &os, const point_2d &a) {
        os << "(" << a._x << ", " << a._y << ")";
        return os;
    }

    class special_fn {
    private:
        point_2d pts[4];
    public:
        special_fn(const point_2d& a, const point_2d &b, const point_2d &c, const point_2d &d)
            : pts{a, b, c, d} {}

        point_2d operator()(node_index_t i) const {
            if (i >= constants::NUM_OF_CUBIC_BEZIER_NODES) {
                throw std::out_of_range("a curve node index is out of range");
            }
            return pts[i];
        }
    };

    pcc_t Cup() {
        return special_fn {point_2d(-1,  1),
                           point_2d(-1, -1),
                           point_2d( 1, -1),
                           point_2d( 1,  1)
        };
    }


    pcc_t Cap() {
        return special_fn {point_2d(-1, -1),
                           point_2d(-1,  1),
                           point_2d( 1,  1),
                           point_2d( 1, -1)
        };
    }

    constexpr real_t ARC = 4 * (std::sqrt(2) - 1) / 3;

    pcc_t ConvexArc() {
        return special_fn {point_2d(0, 1),
                           point_2d(ARC,  1),
                           point_2d(1, ARC),
                           point_2d(1, 0)
        };
    }

    pcc_t ConcaveArc() {
        return special_fn {point_2d(0, 1),
                           point_2d(0, 1 - ARC),
                           point_2d(1 - ARC, 0),
                           point_2d(1, 0)
        };
    }

    pcc_t LineSegment(point_2d p, point_2d q) {
        return special_fn {p, p, q, q};
    }

    pcc_t MovePoint(pcc_t f, node_index_t i, real_t x, real_t y) {
        return [x, y, f, i](node_index_t j){
            return i == j ? f(i) + point_2d(x, y) : f(i);
        };
    }

    pcc_t Rotate(pcc_t f, real_t a) { // friend of point_2d
        return [f, a](node_index_t i){
            return f(i) ; // TODO: rotation wrt to angle a
        };
    }

    pcc_t Scale(pcc_t f, real_t x, real_t y) { // friend of point_2d
        return [f, x, y](node_index_t i){
            return f(i) ; // TODO: not finished (how to do it?)
        };
    }

    pcc_t Translate(pcc_t f, real_t x, real_t y) {
        return [x, y, f, i](node_index_t j){
            return f(i) + point_2d(x, y);
        };
    }

    
    class P3CurvePlotter {
    private:

    public:
        P3CurvePlotter() {}
        Print(std::ostream &os = std::cout, char fb = '*', char bg = ' ') const {}
        point_2d operator()(pcc_t fn, real_t t, node_index_t i) {}
    };

}

#endif //BEZIER_H
