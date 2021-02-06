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

    types::pcc_t special_fn() {
        return [](types::node_index_t)->types::point_2d{
            throw std::out_of_range("a curve node index is out of range");
        };
    }

    template<typename T, typename... Args>
    types::pcc_t special_fn(T t, Args... args) {
        return [t, args...](types::node_index_t i){
            return i == 0 ? t : special_fn(args...)(i - 1);
        };
    }


    types::pcc_t Cup() {
        return special_fn(
              types::point_2d(-1, 1),
              types::point_2d(-1, -1),
              types::point_2d(1, -1),
              types::point_2d(1, 1)
        );
    }


    types::pcc_t Cap() {
        return special_fn(
              types::point_2d(-1, -1),
              types::point_2d(-1, 1),
              types::point_2d(1, 1),
              types::point_2d(1, -1)
        );
    }



    types::pcc_t ConvexArc() {
        return special_fn(
              types::point_2d(0, 1),
              types::point_2d(constants::ARC, 1),
              types::point_2d(1, constants::ARC),
              types::point_2d(1, 0)
        );
    }

    types::pcc_t ConcaveArc() {
        return special_fn(
              types::point_2d(0, 1),
              types::point_2d(0, 1 - constants::ARC),
              types::point_2d(1 - constants::ARC, 0),
              types::point_2d(1, 0)
        );
    }

    types::pcc_t LineSegment(types::point_2d p, types::point_2d q) {
        return special_fn(p, p, q, q);
    }

    types::pcc_t MovePoint(types::pcc_t f, types::node_index_t i, types::real_t x, types::real_t y) {
        return [x, y, f, i](types::node_index_t j){
            return (i == j) ? (f(j) + types::point_2d(x, y)) : f(j);
        };
    }

    types::pcc_t Rotate(types::pcc_t f, types::real_t a) {
        const double sin_a = std::sin(a * constants::PI / 180.0);
        const double cos_a = std::cos(a * constants::PI / 180.0);
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
        constexpr int CBN = constants::NUM_OF_CUBIC_BEZIER_NODES;
        return [f1, f2](types::node_index_t i){
            return i < CBN ? f1(i) : f2(i - CBN);
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
            types::real_t s = (types::real_t)1 / ((types::real_t)k * 2);
            for (int i = 0; i < k; i++) {
                types::point_2d a = (*this)(fn, s * (2 * i + 1), j + 1);
                if (a.X <= 1 && a.X >= -1 && a.Y <= 1 && a.Y >= -1) {

                    set_cell(std::floor((a.X + 1) * _r / 2),
                             std::floor((a.Y + 1) * _r / 2), true);
                }
            }
        }

        types::pcc_t find_point(types::pcc_t fn, types::real_t t, types::node_index_t num) const {
            constexpr int CBN = constants::NUM_OF_CUBIC_BEZIER_NODES;
            types::pcc_t p = (num == CBN - 1 ? fn : find_point(fn, t, num + 1));
            return [t, p](auto i){
                return (1 - t) * p(i) + t * p(i + 1);
            };
        }


    public:
        P3CurvePlotter(types::pcc_t fn, types::node_index_t seg = 1, std::size_t r = 80) : _arr(r * r), _r(r){
            _arr.resize(r * r);
            size_t sq = r * r;
            for (types::node_index_t i = 0; i < seg; i++) {
                draw_segment(fn, sq / seg + (int)(i < (sq % seg)), i);
            }
        }
        void Print(std::ostream &os = std::cout, char fb = '*', char bg = ' ') const {
            for (std::size_t i = 0; i < _r; i++) {
                for (std::size_t j = 0; j < _r; j++) {
                    os << (_arr[ (_r - i - 1) * _r + j ] ? fb : bg);
                }
                os << "\n";
            }

        }
        types::point_2d operator()(types::pcc_t fn, types::real_t t, types::node_index_t i) const {
            constexpr int CBN = constants::NUM_OF_CUBIC_BEZIER_NODES;
            return find_point([i, fn](auto j){ return fn((i - 1) * CBN + j); }, t, 1)(0);
        }
    };

}

#endif //BEZIER_H
