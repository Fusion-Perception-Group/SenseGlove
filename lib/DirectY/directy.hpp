#pragma once

#include <string>
#include <vector>
#include <algorithm>
#include <functional>
#include <tuple>
#include <cmath>
#include <memory>
#include <utility>
#include <cstdint>
#include "ffmt.hpp"
#include "clock.hpp"
#include "font_8x8.hpp"

namespace vms
{
namespace directy
{
    namespace helper
    {
        inline constexpr uint8_t int_sqrt16(const uint16_t x)
        {
            uint8_t res=0;
            uint8_t add= 0x80;   
            for(int i=8; i--;)
            {
                uint8_t temp=res | add;
                uint16_t g2=temp*temp;      
                if (x>=g2)
                {
                    res=temp;           
                }
                add>>=1;
            }
            return res;
        }
        inline constexpr uint16_t int_sqrt32(const uint32_t x)
        {
            if (x <= 0xFFFF)
                return int_sqrt16(x);
            uint16_t res=0;
            uint16_t add= 0x8000;   
            for(int i=16; i--;)
            {
                uint16_t temp=res | add;
                uint32_t g2=temp*temp;      
                if (x>=g2)
                {
                    res=temp;           
                }
                add>>=1;
            }
            return res;
        }

        inline constexpr uint32_t int_sqrt64(const uint64_t x)
        {
            if (x <= 0xFFFFFFFF)
                return int_sqrt32(x);
            uint32_t res=0;
            uint32_t add= 0x80000000;   
            for(int i=32; i--;)
            {
                uint32_t temp=res | add;
                uint64_t g2=temp*temp;      
                if (x>=g2)
                {
                    res=temp;           
                }
                add<<=1;
            }
            return res;
        }

        template <std::unsigned_integral T>
        inline constexpr T int_sqrt(T x)
        requires (4 >= sizeof(T))
        {
            return int_sqrt32(x);
        }

        template <std::unsigned_integral T>
        inline constexpr T int_sqrt(T x)
        requires (sizeof(T) > 4 and 8 >= sizeof(T))
        {
            return int_sqrt64(x);
        }
        
        /**
         * @brief inverse bits order of an integer
         * 
         * @tparam T 
         * @param x 
         * @return constexpr T 
         */
        template <std::unsigned_integral T>
        inline constexpr T inv_bits(T x)
        {
            size_t n = sizeof(T) * 8;
            T mask = ~static_cast<T>(0);
            mask >>= n/2;
            x = ((x >> n/2) & mask) | ((x << n/2) & ~mask);
            n >>= 1;
            for (; n>=1; n>>=1)
            {
                mask = mask & (mask >> n/2);
                mask |= mask << n;
                x = ((x >> n/2) & mask) | ((x << n/2) & ~mask);
            }
            return x;
        }

        template <std::integral T>
        inline constexpr T abs(T x)
        {
            return (x >= 0) ? x : -x;
        }


        /**
         * @brief treat a and b as fixed point from 0.0 to 1.0
         * 
         * @param a 
         * @param b 
         */
        template <std::unsigned_integral T>
        inline T fix_point_mul(T a, T b)
        {
            uint64_t res = static_cast<uint64_t>(a) * b;
            return (res >> (sizeof(T) * 8)) | (res & 1U);
        }

        /**
         * @brief treat a and b as fixed point from 0.0 to 1.0
         * 
         * @param a 
         * @param b 
         */
        template <std::unsigned_integral T>
        inline T fix_point_div(T a, T b)
        {
            return (static_cast<uint64_t>(a) << (sizeof(T) * 8)) / b;
        }
    }
    using coord_t = int32_t;
    using uni_t = uint32_t;

    class Node; // forward declaration
    class BaseCanvas; // forward declaration
    template <typename T=Node> requires std::derived_from<T, Node>
    using NodePtr = std::shared_ptr<T>;
    template <typename T=Node> requires std::derived_from<T, Node>
    using ConstNodePtr = std::shared_ptr<const T>;
    using NodeContainer = std::vector<NodePtr<>>;
    inline constexpr uni_t PX_MAX = std::numeric_limits<uni_t>::max();
    inline constexpr uni_t PX_MIN = std::numeric_limits<uni_t>::min();

    enum class LineType
    {
        Infinite,
        Segment,
        Ray
    };
    enum class TransformRef : uint8_t
    {
        Global,
        Parent,
        Self
    };
    struct Point
    {
        coord_t x, y;
    };
    struct Pixel : public Point
    {
        uni_t color;
        uni_t alpha=PX_MAX;
    };
    struct Context
    {
        coord_t x=0, y=0;
        float sx=1.0f, sy=1.0f; // scale
        float rot=0.0f;  // clockwise, radians
        uni_t alpha=PX_MAX;
        NodePtr<> parent=nullptr;
        uint64_t timestamp_ms=0;
        BaseCanvas &canvas;

        bool no_alpha() const noexcept
        {
            return (alpha == PX_MAX);
        }

        bool no_shift() const noexcept
        {
            return (!x and !y); 
        }

        bool no_scale() const noexcept
        {
            return (sx == 1.0f and sy == 1.0f);
        }

        bool no_rotate() const noexcept
        {
            return (rot == 0.0f);
        }

        bool no_transform() const noexcept
        {
            return no_shift() and no_scale() and no_rotate();
        }

        bool no_modification() const noexcept
        {
            return no_transform() and no_alpha();
        }
    };


    inline constexpr uni_t rescale_color(uni_t raw, const uint8_t px_bits)
    {
        const bool one_ending = (raw & 1U);
        raw <<= (sizeof(uni_t) * 8 - px_bits);  // rescale to full range
        return raw | (raw - one_ending);
    }

    class BaseNode
    {
    protected:
        constexpr BaseNode() {};
        constexpr const BaseNode* _get_raw_ptr() const noexcept { return this; }
    public:
        virtual ~BaseNode() = default;
        NodeContainer children{};
        constexpr bool operator==(const BaseNode &rhs) const
        { return this == rhs._get_raw_ptr(); }

        void add_child(const NodePtr<>& child) { children.push_back(child); }
        void add_child(Node& child);

        size_t insert_child(NodePtr<> child, const size_t index)
        {
            if (index < children.size())
            {
                children.insert(children.begin() + index, child);
                return index;
            }
            children.push_back(child);
            return children.size() - 1;
        }
        size_t insert_child(Node& child, const size_t index);

        bool remove_child(const Node &child);
        bool remove_child(const size_t index)
        {
            if (index < children.size())
            {
                children.erase(children.begin() + index);
                return true;
            }
            return false;
        }

        template <typename T, typename... ARGS>
        T& create_child(ARGS&& ... args) requires std::derived_from<T, Node>;

        template <typename T, typename... ARGS>
        T& create_child_at(size_t index, ARGS&&... args) requires std::derived_from<T, Node>;
    };

    class BaseCanvas; // forward declaration

    class Node : public BaseNode, public std::enable_shared_from_this<Node>
    {
    protected:
        friend class BaseNode;  // friends are not inherited
        constexpr Node() {};
        virtual void _draw_this_impl(const Context &ctx) {}

        static void _draw_pixels_with_ctx(const Context &ctx,
            std::function<std::tuple<Pixel, bool>()> point_gen);

        static Point _transform_coor(const Context &ctx, coord_t x, coord_t y)
        {
            if (ctx.sx != 1.0 or ctx.sy != 1.0)
            {
                x *= ctx.sx;
                y *= ctx.sy;
            }
            if (ctx.rot)
            {
                auto cos_rot = std::cos(ctx.rot);
                auto sin_rot = std::sin(ctx.rot);
                x = x * cos_rot - y * sin_rot;
                y = x * sin_rot + y * cos_rot;
            }
            x += ctx.x;
            y += ctx.y;
            return {x, y};
        }

        static void _draw_pixel_with_ctx(const Context &ctx, coord_t x, coord_t y, uni_t color);
    public:
        bool enabled=true;
        coord_t x=0, y=0;  // xy offset
        float sx=1, sy=1;  // scale
        float rot=0;  // counter-clockwise start from 12:00, radians
        uni_t alpha=PX_MAX;  // alpha channel PX_MAX means opacity = 1.0
        TransformRef transform_ref=TransformRef::Parent;
        NodePtr<> operator&() { return shared_from_this(); }
        ConstNodePtr<> operator&() const { return shared_from_this(); }


        Context draw_this(const Context &ctx)
        {
            Context new_ctx = ctx;
            switch (transform_ref)
            {
                case TransformRef::Self:
                    new_ctx.x += x;
                    new_ctx.y += y;
                    new_ctx.sx = sx;
                    new_ctx.sy = sy;
                    new_ctx.rot = rot;
                    new_ctx.alpha = alpha;
                    break;
                case TransformRef::Parent:
                {
                    new_ctx.rot += rot;
                    new_ctx.alpha = helper::fix_point_mul(alpha, ctx.alpha);
                    new_ctx.sx *= sx;
                    new_ctx.sy *= sy;
                    auto offset_x = x * sx;
                    auto offset_y = y * sy;
                    if (rot)
                    {
                        auto cos_rot = std::cos(rot);
                        auto sin_rot = std::sin(rot);
                        new_ctx.x += offset_x * cos_rot - offset_y * sin_rot;
                        new_ctx.y += offset_x * sin_rot + offset_y * cos_rot;
                    }
                    else
                    {
                        new_ctx.x += offset_x;
                        new_ctx.y += offset_y;
                    }
                    break;
                }
                case TransformRef::Global:
                {
                    new_ctx.rot += rot;
                    new_ctx.alpha = helper::fix_point_mul(alpha, ctx.alpha);
                    new_ctx.sx *= sx;
                    new_ctx.sy *= sy;
                    auto offset_x = x * ctx.sx;
                    auto offset_y = y * ctx.sy;
                    if (ctx.rot)
                    {
                        auto cos_rot = std::cos(ctx.rot);
                        auto sin_rot = std::sin(ctx.rot);
                        new_ctx.x += offset_x * cos_rot - offset_y * sin_rot;
                        new_ctx.y += offset_x * sin_rot + offset_y * cos_rot;
                    }
                    else
                    {
                        new_ctx.x += offset_x;
                        new_ctx.y += offset_y;
                    }
                }
                default:
                    throw std::runtime_error("Invalid transform reference");
            }
            new_ctx.parent = operator&();
            _draw_this_impl(new_ctx);
            return new_ctx;
        }
        void draw(const Context &ctx)
        {
            if (not enabled)
                return;
            auto new_ctx =  draw_this(ctx);
            for (auto &child : children)
                child->draw(new_ctx);
        }
    };

    class BaseCanvas : public BaseNode
    {
        constexpr void _draw_seg(
            coord_t x0, coord_t y0, coord_t x1, coord_t y1,
            const uni_t color, const uni_t alpha=PX_MAX)
        {
            // assume both ends are inside the canvas
            coord_t dx, err=0;
            int step=0;
            if (x0 > x1)  // make sure x0 < x1
            {
                std::swap(x0, x1);
                std::swap(y0, y1);
                dx = x1 - x0;
            }
            else
                dx = x1 - x0;
            coord_t abs_dx = dx;
            coord_t dy = y1 - y0;
            coord_t abs_dy = helper::abs(dy);
            const bool steep = abs_dy > dx;
            const coord_t length = helper::int_sqrt64(dx*dx + dy*dy);
            const coord_t WIDTH = get_width(), HEIGHT = get_height();


            if (!length)
            {
                uni_set_pixel(x0, y0, color, alpha);
                return;
            }

            // if (!dx or !dy)
            // {
            //     // treat horizontal and vertical line as rectangle
            //     uni_draw_rect(x0, y0, x1, y1, color, alpha);  // faster
            //     return;
            // }

            if (steep)  // line is steep, set y as the base
            {
                if (y0 > y1)  // scan from bottom to top
                {
                    std::swap(x0, x1);
                    std::swap(y0, y1);
                    dx = x1 - x0;
                    dy = y1 - y0;
                }
                if (dx)
                    step = (dx > 0) ? 1 : -1;
                for (; y0 <= y1; ++y0, err-=abs_dx)
                {
                    uni_t c_alpha = PX_MAX, l_alpha = PX_MIN, r_alpha = PX_MIN;
                    if (dx)
                    {
                        uni_t abs_err = helper::abs(err);
                        // Quantum mechanics here?
                        // the more accurate of what the color it is,
                        // the less accurate of where it is
                        // auto base = color / length;
                        // unless you use float to preserve more information
                        // but it's slightly more expensive (~2%)
                        uni_t base = PX_MAX / length;
                        c_alpha = base * (length - abs_err);
                        abs_err = helper::abs(err - abs_dy);
                        abs_err = std::min<uni_t>(abs_err, length);
                        l_alpha = base * (length - abs_err);
                        abs_err = helper::abs(err + abs_dy);
                        abs_err = std::min<uni_t>(abs_err, length);
                        r_alpha = base * (length - abs_err);
                        if (step < 0)
                            std::swap(l_alpha, r_alpha);
                    }
                    if (auto x=x0-1; x >= 0)
                    {
                        uni_set_pixel(x, y0, color,
                            helper::fix_point_mul(l_alpha, alpha));
                    }
                    if (auto x=x0+1; x < WIDTH)
                    {
                        uni_set_pixel(x, y0, color,
                            helper::fix_point_mul(r_alpha, alpha));
                    }
                    if (x0 >= 0 and x0 < WIDTH)
                    {
                        uni_set_pixel(x0, y0, color,
                            helper::fix_point_mul(c_alpha, alpha));
                    }
                    if (err <= 0)
                    {
                        x0 += step;
                        err += abs_dy;
                    }
                }
            }
            else
            {
                if (dy)
                    step = (dy > 0) ? 1 : -1;
                for (; x0 <= x1; ++x0, err+=abs_dy)
                {
                    uni_t c_alpha = PX_MAX, u_alpha = PX_MIN, d_alpha = PX_MIN;
                    if (dy)
                    {
                        auto abs_err = helper::abs(err);
                        uni_t base = PX_MAX / length;
                        c_alpha = base * (length - abs_err);
                        abs_err = helper::abs(err - abs_dx);
                        abs_err = std::min<uni_t>(abs_err, length);
                        u_alpha = base * (length - abs_err);
                        abs_err = helper::abs(err + abs_dx);
                        abs_err = std::min<uni_t>(abs_err, length);
                        d_alpha = base * (length - abs_err);
                        if (step < 0)
                            std::swap(u_alpha, d_alpha);
                    }
                    if (auto y=y0-1; y >= 0)
                    {
                        uni_set_pixel(x0, y, color,
                            helper::fix_point_mul(d_alpha, alpha));
                    }
                    if (auto y=y0+1; y < HEIGHT)
                    {
                        uni_set_pixel(x0, y,
                            helper::fix_point_mul(u_alpha, alpha));
                    }
                    uni_set_pixel(x0, y0,
                        helper::fix_point_mul(c_alpha, alpha));
                    if (err >= 0)
                    {
                        y0 += step;
                        err -= abs_dx;
                    }
                }
            }
        }
    public:
        BaseCanvas() = default;

        constexpr virtual coord_t get_width() const = 0;
        constexpr virtual coord_t get_height() const = 0;
        constexpr virtual uint8_t get_pixel_bits() const = 0;

        virtual void draw(
            coord_t x_offset=0, coord_t y_offset=0,
            float x_scale=1, float y_scale=1, float rotation=0,
            uni_t alpha=PX_MAX,
            uint64_t timestamp_ms=stm32::clock::get_systick_ms())
        {
            Context ctx{
                .x = x_offset,
                .y = y_offset,
                .sx = x_scale,
                .sy = y_scale,
                .rot = rotation,
                .alpha = alpha,
                .parent = nullptr,
                .timestamp_ms = timestamp_ms,
                .canvas = *this
            };
            for (auto &child : children)
                child->draw(ctx);
        }

        constexpr virtual uni_t uni_get_pixel(coord_t x, coord_t y) const = 0;
        constexpr virtual void uni_set_pixel(
            coord_t x, coord_t y, uni_t color, uni_t alpha=PX_MAX) = 0;

        constexpr virtual void uni_fill(uni_t color)
        {
            const auto WIDTH = get_width(), HEIGHT = get_height();
            for (coord_t y=0; y < HEIGHT; ++y)
            {
                for (coord_t x=0; x < WIDTH; ++x)
                {
                    uni_set_pixel(x, y, color);
                }
            }
        }

        constexpr virtual void clear()
        {
            uni_fill(PX_MIN);
        }

        constexpr virtual void uni_draw_line(
            coord_t x0, coord_t y0, coord_t x1, coord_t y1,
            uni_t color=PX_MAX, uni_t alpha=PX_MAX,
            LineType type=LineType::Segment)
        {
            const coord_t HEIGHT = get_height(), WIDTH = get_width();
            coord_t dx = x1 - x0;
            coord_t dy = y1 - y0;
            unsigned inter_cnt = 0;
            coord_t inter_sects[2][2];

            if (x0 >= 0 and x0 < WIDTH
                and y0 >= 0 and y0 < HEIGHT
                and x1 >= 0 and x1 < WIDTH
                and y1 >= 0 and y1 < HEIGHT
                and type == LineType::Segment)
            {
                _draw_seg(x0, y0, x1, y1, color);
                return;
            }

            if (not dx and not dy)
            {
                throw std::invalid_argument(
                    "Unit length line only allowed in line segment mode");
            }

            if (not dx)  // prevent division by zero
            {
                if (x0 >= 0 and x0 < WIDTH)  // vertical line
                {
                    inter_cnt = 2;
                    inter_sects[0][0] = x0;
                    inter_sects[0][1] = 0;
                    inter_sects[1][0] = x0;
                    inter_sects[1][1] = HEIGHT - 1;
                }
            }
            else if (not dy)  // prevent division by zero
            {
                if (y0 >= 0 and y0 < HEIGHT)  // horizontal line
                {
                    inter_cnt = 2;
                    inter_sects[0][0] = 0;
                    inter_sects[0][1] = y0;
                    inter_sects[1][0] = WIDTH - 1;
                    inter_sects[1][1] = y0;
                }
            }
            else
            {
                coord_t tmp;
                // intersection with left edge
                tmp = (-x0 * dy / dx) + y0;
                if (tmp >= 0 and tmp < HEIGHT)
                {
                    inter_sects[inter_cnt][0] = 0;
                    inter_sects[inter_cnt][1] = tmp;
                    ++inter_cnt;
                }
                // intersection with right edge
                tmp = ((WIDTH - 1 - x0) * dy / dx) + y0;
                if (tmp >= 0 and tmp < HEIGHT)
                {
                    inter_sects[inter_cnt][0] = WIDTH - 1;
                    inter_sects[inter_cnt][1] = tmp;
                    ++inter_cnt;
                }
                if (inter_cnt != 2)  // may have found two intersections
                {
                    // intersection with top edge
                    tmp = (-y0 * dx / dy) + x0;
                    if (tmp >= 0 and tmp < WIDTH
                        and not (inter_cnt  // prevent duplicate points
                                and inter_sects[0][0] == tmp
                                and inter_sects[0][1] == 0))
                    {
                        inter_sects[inter_cnt][0] = tmp;
                        inter_sects[inter_cnt][1] = 0;
                        ++inter_cnt;
                    }
                }
                if (inter_cnt != 2)  // may have found two intersections
                {
                    // intersection with bottom edge
                    tmp = ((HEIGHT - 1 - y0) * dx / dy) + x0;
                    if (tmp >= 0 and tmp < WIDTH)
                    {
                        inter_sects[inter_cnt][0] = tmp;
                        inter_sects[inter_cnt][1] = HEIGHT - 1;
                        ++inter_cnt;
                    }
                }
            }

            if (inter_cnt != 2)
                return;  // line outside the canvas
            
            if (type == LineType::Infinite)
                return _draw_seg(
                    inter_sects[0][0], inter_sects[0][1],
                    inter_sects[1][0], inter_sects[1][1], color, alpha);
            
            if (type == LineType::Segment)
            {
                // if segment type ever reach here, then at least one
                // end is outside the canvas
                if (x1 >= 0 and x1 < WIDTH
                    and y1 >= 0 and y1 < HEIGHT)
                {
                    // (x1, y1) inside the canvas
                    x0 = x1, y0 = y1;
                }
                // otherwise, (x0, y0) is inside the canvas
            }

            // treat segment and ray the same way
            if (x0 >= 0 and x0 < WIDTH and y0 >= 0 and y0 < HEIGHT)
            {
                // (x0, y0) inside the canvas
                if ((x0 != inter_sects[0][0] or y0 != inter_sects[0][1])
                    and dx * (inter_sects[0][0]-x0) >=0
                    and dy * (inter_sects[0][1]-y0) >=0)
                {
                    // intersection[0] inside of the ray
                    return _draw_seg(
                        x0, y0,
                        inter_sects[0][0], inter_sects[0][1], color, alpha);
                }
                // intersection[1] inside of the ray
                return _draw_seg(
                    x0, y0,
                    inter_sects[1][0], inter_sects[1][1], color, alpha);
            }
            // origin of the ray is outside
            return _draw_seg(
                inter_sects[0][0], inter_sects[0][1],
                inter_sects[1][0], inter_sects[1][1], color, alpha);

        }

        /**
         * @brief draw a not filled rectangle
         * 
         * @param x0 left buttom corner x
         * @param y0 left buttom corner y
         * @param w width
         * @param h height
        (void)&*child;
         * @param color 
         */
        constexpr virtual void uni_draw_rect(
            coord_t x0, coord_t y0, coord_t x1, coord_t y1,
            uni_t color=PX_MAX, uni_t alpha=PX_MAX)
        {
            const auto WIDTH = get_width(), HEIGHT = get_height();
            if (x0 > x1)  // make sure x0 < x1
            {
                std::swap(x0, x1);
            }
            if (y0 > y1)  // make sure y0 < y1
            {
                std::swap(y0, y1);
            }
            bool has_l = (x0 >= 0 and x0 < WIDTH);
            bool has_r = (x1 >= 0 and x1 < WIDTH and x1 != x0);
            bool has_t = (y0 >= 0 and y0 < HEIGHT);
            bool has_b = (y1 >= 0 and y1 < HEIGHT and y1 != y0);
            x0 = std::clamp<coord_t>(x0, 0, WIDTH-1);
            x1 = std::clamp<coord_t>(x1, 0, WIDTH-1);
            y0 = std::clamp<coord_t>(y0, 0, HEIGHT-1);
            y1 = std::clamp<coord_t>(y1, 0, HEIGHT-1);
            if (has_t)
                for (coord_t x=x0; x <= x1; ++x)
                {
                    uni_set_pixel(x, y1, color, alpha);
                }
            if (has_b)
                for (coord_t x=x0; x <= x1; ++x)
                {
                    uni_set_pixel(x, y0, color, alpha);
                }
            if (has_l)
                for (coord_t y=y0; y <= y1; ++y)
                {
                    uni_set_pixel(x0, y, color, alpha);
                }
            if (has_r)
                for (coord_t y=y0; y <= y1; ++y)
                {
                    uni_set_pixel(x1, y, color, alpha);
                }
        }
        constexpr virtual void uni_draw_filled_rect(
            coord_t x0, coord_t y0, coord_t x1, coord_t y1,
            uni_t color=PX_MAX, uni_t alpha=PX_MAX)
        {
            const auto WIDTH = get_width(), HEIGHT = get_height();
            if (x0 > x1)  // make sure x0 < x1
            {
                std::swap(x0, x1);
            }
            if (y0 > y1)  // make sure y0 < y1
            {
                std::swap(y0, y1);
            }
            x0 = std::clamp<coord_t>(x0, 0, WIDTH-1);
            x1 = std::clamp<coord_t>(x1, 0, WIDTH-1);
            y0 = std::clamp<coord_t>(y0, 0, HEIGHT-1);
            y1 = std::clamp<coord_t>(y1, 0, HEIGHT-1);
            for (coord_t y=y0; y <= y1; ++y)
            {
                for (coord_t x=x0; x <= x1; ++x)
                {
                    uni_set_pixel(x, y, color, alpha);
                }
            }
        }

        /**
         * @brief draw a circle
         * 
         * @param x0 
         * @param y0 
         * @param r inner radius
         * @param color 
         */
        constexpr virtual void uni_draw_circle(
            coord_t x0, coord_t y0, coord_t r,
            uni_t color=PX_MAX, uni_t alpha=PX_MAX)
        {
            const auto WIDTH = get_width(), HEIGHT = get_height();
            r = helper::abs(r);
            if (x0 < -r or x0 > WIDTH + r or y0 < -r or y0 > HEIGHT + r)
                return;
            coord_t x = r, y = 0;
            coord_t err = 1-x;
            while (x >= y)
            {
                uni_set_pixel(x0 + x, y0 + y, color, alpha);
                uni_set_pixel(x0 + y, y0 + x, color, alpha);
                uni_set_pixel(x0 - y, y0 + x, color, alpha);
                uni_set_pixel(x0 - x, y0 + y, color, alpha);
                uni_set_pixel(x0 - x, y0 - y, color, alpha);
                uni_set_pixel(x0 - y, y0 - x, color, alpha);
                uni_set_pixel(x0 + y, y0 - x, color, alpha);
                uni_set_pixel(x0 + x, y0 - y, color, alpha);
                ++y;
                if (err < 0)
                {
                    err += 2 * y + 1;
                }
                else
                {
                    --x;
                    err += 2 * (y - x + 1);
                }
            }
        }
    };


    inline void BaseNode::add_child(Node& child) { add_child(&child); }
    size_t BaseNode::insert_child(Node& child, const size_t index)
        { return insert_child(&child, index); }
    inline bool BaseNode::remove_child(const Node &child)
        {
            auto it = std::find(children.begin(), children.end(), &child);
            if (it != children.end())
            {
                children.erase(it);
                return true;
            }
            return false;
        }
    template <typename T, typename... ARGS>
    inline T& BaseNode::create_child(ARGS&& ... args)
    requires std::derived_from<T, Node>
    {
        auto child = NodePtr<T>(new T(std::forward<ARGS>(args)...));
        add_child(child);
        return *child;
    }
    template <typename T, typename... ARGS>
    inline T& BaseNode::create_child_at(size_t index, ARGS&& ... args)
    requires std::derived_from<T, Node>
    {
        auto child = NodePtr<T>(new T(std::forward<ARGS>(args)...));
        insert_child(child, index);
        return *reinterpret_cast<T*>(child.get());
    }

    inline void Node::_draw_pixel_with_ctx(const Context &ctx, coord_t x, coord_t y, uni_t color)
    {
        auto p = _transform_coor(ctx, x, y);
        x = p.x;
        y = p.y;
        if (x < 0 or x >= ctx.canvas.get_width()
            or y < 0 or y >= ctx.canvas.get_height())
            return;
        ctx.canvas.uni_set_pixel(x, y, color, ctx.alpha);
    }

    inline void Node::_draw_pixels_with_ctx(const Context &ctx,
            std::function<std::tuple<Pixel, bool>()> point_gen)
        {
            auto cos_rot = 1.0f;
            auto sin_rot = 0.0f;
            if (ctx.rot)
            {
                cos_rot = std::cos(ctx.rot);
                sin_rot = std::sin(ctx.rot);
            }
            bool last;
            do {
                auto t = point_gen();
                auto pixel = std::get<0>(t);
                last = std::get<1>(t);
                auto x = pixel.x, y = pixel.y;
                auto color = pixel.color;
                if (not ctx.no_scale())
                {
                    x *= ctx.sx;
                    y *= ctx.sy;
                }
                if (ctx.rot)
                {
                    x = x * cos_rot - y * sin_rot;
                    y = x * sin_rot + y * cos_rot;
                }
                x += ctx.x;
                y += ctx.y;
                if (x < 0 or x >= ctx.canvas.get_width()
                    or y < 0 or y >= ctx.canvas.get_height())
                    continue;
                ctx.canvas.uni_set_pixel(x, y, color, ctx.alpha);
            } while (not last);
        }

    /**
     * @brief Canvas with fixed size
     * 
     * @tparam W Width
     * @tparam H Height
     * @tparam BIT Bits per pixel
     */
    template <size_t W, size_t H, uint8_t BIT, typename PX_T=uni_t>
    requires (BIT != 0 and BIT <= sizeof(uni_t) * 8)
    class Canvas : public BaseCanvas
    {
        using Mat_t = std::array<std::array<PX_T, W>, H>;
    public:
        Mat_t pixels{};
        constexpr coord_t get_width() const override { return W; }
        constexpr coord_t get_height() const override { return H; }
        constexpr uint8_t get_pixel_bits() const override { return BIT; }

        constexpr void uni_fill(uni_t color) override
        {
            color >>= (sizeof(uni_t) * 8 - BIT);  // rescale to pixel range
            for (auto &row : pixels)
                row.fill(color);  // rescale to pixel range
        }

        constexpr uni_t uni_get_pixel(const coord_t x, const coord_t y) const override
        {
            if (x < 0 or x >= static_cast<coord_t>(W) or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            auto pixel = pixels[y][x];
            return rescale_color(pixel, BIT);
        }
        constexpr void uni_set_pixel(
            const coord_t x, const coord_t y,
            uni_t color=PX_MAX, const uni_t alpha=PX_MAX) override
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            if (alpha != PX_MAX)
            {
                auto bg_color = uni_get_pixel(x, y);
                color = helper::fix_point_mul(color, alpha)
                    + helper::fix_point_mul(bg_color, PX_MAX - alpha);
            }
            pixels[y][x] = color >> (sizeof(uni_t) * 8 - BIT);  // rescale to pixel range
        }

        constexpr PX_T get_pixel(const coord_t x, const coord_t y) const
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            return pixels[y][x];
        }
        constexpr void set_pixel(const coord_t x, const coord_t y, const PX_T color)
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            pixels[y][x] = color;
        }
    };

    template <size_t W, size_t H, uint8_t BIT>
    requires (BIT != 0 and BIT <= 4)
    class Canvas4B : public BaseCanvas
    {
        using Mat_t = std::array<std::array<uint8_t, W>, (H+1)/2>;
    public:
        Mat_t pixels{};
        constexpr coord_t get_width() const override { return W; }
        constexpr coord_t get_height() const override { return H; }
        constexpr uint8_t get_pixel_bits() const override { return BIT; }
        
        constexpr void uni_fill(uni_t color) override
        {
            color >>= (sizeof(uni_t) * 8 - BIT);  // rescale to pixel range
            color |= color << 4;
            for (auto &row : pixels)
                row.fill(color);  // rescale to pixel range
        }

        constexpr uni_t uni_get_pixel(
            const coord_t x, const coord_t y) const override
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            uni_t pixel = pixels[y/2][x];
            pixel = (y % 2) ? (pixel & 0x0F) : (pixel >> 4);
            return rescale_color(pixel, BIT);
        }
        constexpr void uni_set_pixel(
            const coord_t x, const coord_t y,
            uni_t color=PX_MAX, const uni_t alpha=PX_MAX) override
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            if (alpha != PX_MAX)
            {
                auto bg_color = uni_get_pixel(x, y);
                color = helper::fix_point_mul(color, alpha)
                    + helper::fix_point_mul(bg_color, PX_MAX - alpha);
            }
            uni_t pixel = color >> (sizeof(uni_t) * 8 - BIT);  // rescale to pixel range
            if (y % 2)
                pixels[y/2][x] = (pixels[y/2][x] & 0xF0) | pixel;
            else
                pixels[y/2][x] = (pixels[y/2][x] & 0x0F) | (pixel << 4);
        }

        constexpr uint8_t get_pixel(const coord_t x, const coord_t y) const
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            return (y % 2) ? (pixels[y/2][x] & 0x0F) : (pixels[y/2][x] >> 4);
        }
        constexpr void set_pixel(const coord_t x, const coord_t y, const uint8_t color)
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            if (y % 2)
                pixels[y/2][x] = (pixels[y/2][x] & 0xF0) | color;
            else
                pixels[y/2][x] = (pixels[y/2][x] & 0x0F) | (color << 4);
        }
    };

    template <size_t W, size_t H, uint8_t BIT>
    requires (BIT != 0 and BIT <= 2)
    class Canvas2B : public BaseCanvas
    {
        using Mat_t = std::array<std::array<uint8_t, W>, (H+3)/4>;
    public:
        Mat_t pixels{};
        constexpr coord_t get_width() const override { return W; }
        constexpr coord_t get_height() const override { return H; }
        constexpr uint8_t get_pixel_bits() const override { return BIT; }

        constexpr void uni_fill(uni_t color) override
        {
            color = (color >> (sizeof(uni_t) * 8 - BIT));  // rescale to pixel range
            color |= color << 2 | color << 4 | color << 6;
            for (auto &row : pixels)
                row.fill(color);  // rescale to pixel range
        }

        constexpr uni_t uni_get_pixel(const coord_t x, const coord_t y) const override
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            uni_t pixel = pixels[y/4][x];
            const unsigned shift = (y % 4) * 2;
            pixel = (pixel >> shift) & 0x03;
            return rescale_color(pixel, BIT);
        }
        constexpr void uni_set_pixel(
            const coord_t x, const coord_t y,
            uni_t color=PX_MAX, const uni_t alpha=PX_MAX) override
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            if (alpha != PX_MAX)
            {
                auto bg_color = uni_get_pixel(x, y);
                color = helper::fix_point_mul(color, alpha)
                    + helper::fix_point_mul(bg_color, PX_MAX - alpha);
            }
            uni_t pixel = color >> (sizeof(uni_t) * 8 - BIT);  // rescale to pixel range
            const unsigned shift = (y % 4) * 2;
            pixels[y/4][x] = (pixels[y/4][x] & ~(0x03 << shift)) | (pixel << shift);
        }

        constexpr uint8_t get_pixel(const coord_t x, const coord_t y) const
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            const unsigned shift = (y % 4) * 2;
            return (pixels[y/4][x] >> shift) & 0x03;
        }
        constexpr void set_pixel(const coord_t x, const coord_t y, const uint8_t color)
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            const unsigned shift = (y % 4) * 2;
            pixels[y/4][x] = (pixels[y/4][x] & ~(0x03 << shift)) | (color << shift);
        }
    };

    template <size_t W, size_t H, uint8_t BIT=1>
    requires (BIT == 1)
    class Canvas1B : public BaseCanvas
    {
        using Mat_t = std::array<std::array<uint8_t, W>, (H+7)/8>;
    public:
        Mat_t pixels{};
        constexpr coord_t get_width() const override { return W; }
        constexpr coord_t get_height() const override { return H; }
        constexpr uint8_t get_pixel_bits() const override { return BIT; }

        constexpr void uni_fill(uni_t color) override
        {
            color = (color & 0x80U) ? 0xffU : 0x00U;  // rescale to pixel range
            for (auto &row : pixels)
                row.fill(color);  // rescale to pixel range
        }

        constexpr uni_t uni_get_pixel(const coord_t x, const coord_t y) const override
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            uni_t pixel = pixels[y/8][x];
            const unsigned shift = y % 8;
            pixel = (pixel >> shift) & 0x01;
            return rescale_color(pixel, BIT);
        }
        constexpr void uni_set_pixel(
            const coord_t x, const coord_t y,
            uni_t color=PX_MAX, const uni_t alpha=PX_MAX) override
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            if (alpha != PX_MAX)
            {
                auto bg_color = uni_get_pixel(x, y);
                color = helper::fix_point_mul(color, alpha)
                    + helper::fix_point_mul(bg_color, PX_MAX - alpha);
            }
            uni_t pixel = color >> (sizeof(uni_t) * 8 - BIT);  // rescale to pixel range
            const unsigned shift = y % 8;
            pixels[y/8][x] = (pixels[y/8][x] & ~(0x01 << shift)) | (pixel << shift);
        }

        constexpr bool get_pixel(const coord_t x, const coord_t y) const
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            const unsigned shift = y % 8;
            return (pixels[y/8][x] >> shift) & 0x01;
        }
        constexpr void set_pixel(const coord_t x, const coord_t y, const bool color)
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            const unsigned shift = y % 8;
            pixels[y/8][x] = ((pixels[y/8][x] & ~(0x01 << shift))
                                | (static_cast<uint8_t>(color) << shift));
        }
    };

    class LineNode : public Node
    {
    protected:
        friend class BaseNode;
        using Node::Node;
        constexpr LineNode(
            coord_t x0=0, coord_t y0=0, coord_t x1=0, coord_t y1=0,
            uni_t color=PX_MAX, LineType type=LineType::Segment)
            : Node(), x0(x0), y0(y0), x1(x1), y1(y1), color(color), type(type) {};
        void _draw_this_impl(const Context &ctx) override
        {
            auto _x0 = x0, _y0 = y0, _x1 = x1, _y1 = y1;
            if (ctx.rot)
            {
                auto cos_rot = std::cos(ctx.rot);
                auto sin_rot = std::sin(ctx.rot);
                _x0 = x0 * cos_rot - y0 * sin_rot;
                _y0 = x0 * sin_rot + y0 * cos_rot;
                _x1 = x1 * cos_rot - y1 * sin_rot;
                _y1 = x1 * sin_rot + y1 * cos_rot;
            }
            if (ctx.sx != 1 or ctx.sy != 1)
            {
                _x0 *= ctx.sx;
                _y0 *= ctx.sy;
                _x1 *= ctx.sx;
                _y1 *= ctx.sy;
            }
            ctx.canvas.uni_draw_line(
                ctx.x + _x0, ctx.y + _y0,
                ctx.x + _x1, ctx.y + _y1, color, alpha, type);
        }
    public:
        coord_t x0, y0, x1, y1;
        uni_t color;
        LineType type;
    };

    class TextBoxNode : public Node
    {
    protected:
        friend class BaseNode;
        using Node::Node;
        constexpr TextBoxNode(std::string_view content,
            const coord_t x=0, const coord_t y=0,
            const coord_t w=UNSET_HW, const coord_t h=UNSET_HW)
            : Node(), content(content), width(w), height(h)
            {
                this->x = x;
                this->y = y;
            }
        void _draw_this_impl(const Context &ctx) override
        {
            // temporary implementation using 8x8 font
            auto font = shared::FONT_8x8;
            const coord_t WIDTH = ctx.canvas.get_width();
            const coord_t HEIGHT = ctx.canvas.get_height();
            auto x0 = ctx.x, y0 = ctx.y;
            const coord_t w = (width < 0) ? WIDTH - x0 : width;
            const coord_t h = (height < 0) ? HEIGHT - y0 : height;
            const coord_t x1 = x0 + w - 1, y1 = y0 + h - 1;
            coord_t x = x0 + margin, y = y1 - margin;
            if (transform_ref != TransformRef::Self
                and (x0 >= WIDTH or y0 >= HEIGHT
                    or w - margin*2 < 8 or h - margin*2 < 8
                    or x1 <= 0 or y1 <= 0))
                return;
            
            unsigned tab_cnt = 4;
            for (size_t i=0, MAX=content.size(); i < MAX; ++i)
            {
                auto c = content[i];
                if (x + static_cast<coord_t>(font_size) - 1
                    > x1 - static_cast<coord_t>(margin))  // line exceeding
                {
                    switch (exceed_action)
                    {
                        case ExceedAction::Truncate:
                            for (; i < MAX; ++i)
                            {
                                if (content[i] == '\n')
                                {  // find next new line
                                    if (++i >= MAX)
                                        return;  // no more content
                                    c = content[i];
                                    break;
                                }
                                return;
                            }  // no break here
                        case ExceedAction::Wrap:
                            x = x0 + margin;
                            y -= font_size + line_spacing;
                            break;
                        default:
                            throw std::runtime_error("Invalid exceed action");
                    }
                }
                if (y <= y0 + static_cast<coord_t>(margin))  // out of box
                    return;
                switch (c)
                {
                    case '\n':
                    case '\r':
                        x = x0 + margin;
                        y -= font_size + line_spacing;
                        break;
                    case '\t':
                        if (--tab_cnt)
                            --i;  // keep current character
                        // no break here
                    default:
                    {
                        if (static_cast<int>(c) >= 256)
                            c = '?';
                        auto glyph = font[static_cast<uint8_t>(c)];
                        for (coord_t col=0; col < 8; ++col)
                        {
                            uint8_t glyph_col = glyph[col];
                            for (coord_t row=0; row < 8; ++row)
                            {
                                auto color = ((glyph_col >> row) & 0x01) ? PX_MAX : PX_MIN;
                                auto px_x = x + col;
                                auto px_y = y - row;
                                if (px_x >= x0 and px_x < x1
                                    and px_y >= y0 and px_y < y1)
                                {
                                    _draw_pixel_with_ctx(ctx, px_x, px_y, color);
                                }
                            }
                        }
                        x += font_size;
                    }
                }
            }
        }
    public:
        enum class ExceedAction
        {
            Wrap,
            Truncate
        };
        static constexpr const coord_t UNSET_HW = -1;  // unset value for height and width
        std::string content{};
        size_t font_size=8;
        size_t line_spacing=2;
        size_t margin = 0;
        coord_t width=UNSET_HW, height=UNSET_HW;
        ExceedAction exceed_action=ExceedAction::Wrap;
    };

}
}
