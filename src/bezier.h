#ifndef BEZIER_H
#define BEZIER_H

#include <iostream>
#include <functional>
#include <cmath>
#include <initializer_list>
#include <vector>

namespace bezier {

    namespace types {
        class point_2d;
        class special_fn;

        using real_t = double;
        using node_index_t = unsigned int;
        using pcc_t = std::function<point_2d(node_index_t)>;
    }

    namespace constants {
        constexpr int NUM_OF_CUBIC_BEZIER_NODES = 4;
        constexpr types::real_t ARC = 4 * (std::sqrt(2) - 1) / 3;
        constexpr double PI = std::acos(-1);
    }

    class types::point_2d {
    public:
        point_2d(real_t x, real_t y) : X(x), Y(y) {}

        const real_t X;
        const real_t Y;
    };

    namespace types {
        point_2d operator+(const point_2d &a, const point_2d &b) {
            return point_2d(a.X + b.X, a.Y + b.Y);
        }

        point_2d operator*(const point_2d &a, real_t r) {
            return point_2d(a.X * r, a.Y * r);
        }

        point_2d operator*(real_t r, const point_2d &a) {
            return a * r;
        }

        bool operator==(const point_2d &a, const point_2d &b) {
            return a.X == b.X && a.Y == b.Y;
        }

        std::ostream &operator<<(std::ostream &os, const point_2d &a) {
            os << "(" << a.X << ", " << a.Y << ")";
            return os;
        }
    }

    // TODO: change this function so that it uses the constant

    class types::special_fn {
    private:
        const std::vector<point_2d> _pts;
    public:
        special_fn(std::initializer_list<point_2d> pts): _pts{pts} {
            if (_pts.size() != constants::NUM_OF_CUBIC_BEZIER_NODES) {
                throw std::invalid_argument("a wrong number of cubic bezier nodes");
            }
        }

        point_2d operator()(node_index_t i) const {
            if (i >= constants::NUM_OF_CUBIC_BEZIER_NODES) {
                throw std::out_of_range("a curve node index is out of range");
            }
            return _pts[i];
        }
    };

    types::pcc_t Cup() {
        return types::special_fn {{
              types::point_2d(-1, 1),
              types::point_2d(-1, -1),
              types::point_2d(1, -1),
              types::point_2d(1, 1)
        }};
    }


    types::pcc_t Cap() {
        return types::special_fn {{
              types::point_2d(-1, -1),
              types::point_2d(-1, 1),
              types::point_2d(1, 1),
              types::point_2d(1, -1)
        }};
    }



    types::pcc_t ConvexArc() {
        return types::special_fn {{
              types::point_2d(0, 1),
              types::point_2d(constants::ARC, 1),
              types::point_2d(1, constants::ARC),
              types::point_2d(1, 0)
        }};
    }

    types::pcc_t ConcaveArc() {
        return types::special_fn {{
              types::point_2d(0, 1),
              types::point_2d(0, 1 - constants::ARC),
              types::point_2d(1 - constants::ARC, 0),
              types::point_2d(1, 0)
        }};
    }

    types::pcc_t LineSegment(types::point_2d p, types::point_2d q) {
        return types::special_fn{{p, p, q, q}};
    }

    types::pcc_t MovePoint(types::pcc_t f, types::node_index_t i, types::real_t x, types::real_t y) {
        return [x, y, f, i](types::node_index_t j){
            return i == j ? f(i) + types::point_2d(x, y) : f(i);
        };
    }

    types::pcc_t Rotate(types::pcc_t f, types::real_t a) {
        const double sin_a = std::sin(a / (2 * constants::PI));
        const double cos_a = std::cos(a / (2 * constants::PI));
        return [f, sin_a, cos_a](types::node_index_t i){
            const types::point_2d p = f(i);
            return types::point_2d(p.X * cos_a - p.Y * sin_a, p.X * sin_a + p.Y * cos_a);
        };
    }

    types::pcc_t Scale(types::pcc_t f, types::real_t x, types::real_t y) {
        return [f, x, y](types::node_index_t i){
            types::point_2d a = f(i);
            return types::point_2d(a.X * x, a.Y * y);
        };
    }

    types::pcc_t Translate(types::pcc_t f, types::real_t x, types::real_t y) {
        return [x, y, f](types::node_index_t i){
            return f(i) + types::point_2d(x, y);
        };
    }

    types::pcc_t operator*(types::pcc_t f1, types::pcc_t f2) {
        return [f1, f2](types::node_index_t i){
            if (i < constants::NUM_OF_CUBIC_BEZIER_NODES) {
                return f1(i);
            } else {
                return f2(i - constants::NUM_OF_CUBIC_BEZIER_NODES);
            }
        };
    }

    template<typename ...Args>
    types::pcc_t Concatenate(Args ...fns) {
        return (fns * ...);
    }

    class P3CurvePlotter {
    private:
        std::vector<bool> _arr;
        std::size_t _r;

        void set_cell(std::size_t x, std::size_t y, bool b) {
            if (x < _r && y < _r) {
                _arr[y * _r + x] = b;
            }
        }

        void draw_segment(types::pcc_t fn, int k, int j) {
            types::real_t s = (types::real_t)2 / (types::real_t)k;
            for (int i = 0; i < k; i++) {
                types::point_2d a = (*this)(fn, s * i - 1, j);
                if (a.X <= 1 && a.X >= -1 && a.Y <= 1 && a.Y >= -1) {

                    set_cell((a.X + 1) * _r / 2, (a.Y + 1) * _r / 2, true);
                }
            }
        }

        types::pcc_t find_point(types::pcc_t fn, types::real_t t, types::node_index_t num) {
            using constants::NUM_OF_CUBIC_BEZIER_NODES;

            types::pcc_t p;
            if (num == NUM_OF_CUBIC_BEZIER_NODES) {
                p = fn;
            } else {
                p = find_point(fn, t, num + 1);
            }

            return [t, p](types::node_index_t i){
                return (1 - t) * p(i) + t * p(i + 1);
            };
        }


    public:
        P3CurvePlotter(types::pcc_t fn, types::node_index_t seg = 1, std::size_t r = 80) : _arr(r * r), _r(r){
            _arr.resize(r * r);
            for (types::node_index_t i = 0; i < seg; i++) {
                draw_segment(fn, r * r / seg, i);
            }
        }
        void Print(std::ostream &os = std::cout, char fb = '*', char bg = ' ') const {
            for (std::size_t i = 0; i < _r * _r; i++) {
                os << (_arr[i] ? fb : bg);
                if (i % _r == _r - 1) {
                    os << "\n";
                }
            }
            std::cout << "DONE" << std::endl;
        }
        types::point_2d operator()(types::pcc_t fn, types::real_t t, types::node_index_t i) {
            using constants::NUM_OF_CUBIC_BEZIER_NODES;
            types::pcc_t p = [i, fn](types::node_index_t j){
                return fn(i * NUM_OF_CUBIC_BEZIER_NODES + j);
            };
            return find_point(p, t, 1)(0);
        }
    };

}

#endif //BEZIER_H
