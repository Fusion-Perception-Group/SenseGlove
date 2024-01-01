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

namespace vermils
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
    };
    struct Context
    {
        TransformRef ref=TransformRef::Parent;
        coord_t x=0, y=0;
        float sx=1.0f, sy=1.0f; // scale
        float rot=0.0f;  // clockwise, radians
        NodePtr<> parent=nullptr;
        uint64_t timestamp_ms=0;
        BaseCanvas &canvas;
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
        void _draw_points_with_ctx(const Context &ctx,
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
                if (ctx.sx != 1.0 or ctx.sy != 1.0)
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
                ctx.canvas.uni_set_pixel(x, y, color);
            } while (not last);
        }
        void _draw_point_with_ctx(const Context &ctx, coord_t x, coord_t y, uni_t color)
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
            ctx.canvas.uni_set_pixel(x, y, color);
        }
    public:
        bool enabled=true;
        coord_t x=0, y=0;  // xy offset
        float sx=1, sy=1;  // scale
        float rot=0;  // counter-clockwise start from 12:00, radians
        NodePtr<> operator&() { return shared_from_this(); }
        ConstNodePtr<> operator&() const { return shared_from_this(); }


        Context draw_this(const Context &ctx)
        {
            Context new_ctx = ctx;
            if (sx != 1.0 or sy != 1.0)
            {
                new_ctx.sx *= sx;
                new_ctx.sy *= sy;
            }
            new_ctx.rot += rot;
            switch (ctx.ref)
            {
                case TransformRef::Self:
                    new_ctx.x += x;
                    new_ctx.y += y;
                    break;
                case TransformRef::Parent:
                {
                    auto offset_x = (sx != 1.0) ? x * sx : x;
                    auto offset_y = (sy != 1.0) ? y * sy : y;
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
                    auto offset_x = (ctx.sx != 1.0) ? x * ctx.sx : x;
                    auto offset_y = (ctx.sy != 1.0) ? y * ctx.sy : y;
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
            coord_t x0, coord_t y0, coord_t x1, coord_t y1, const uni_t color)
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
                uni_set_pixel(x0, y0, color);
                return;
            }

            // if (!dx or !dy)
            // {
            //     // treat horizontal and vertical line as rectangle
            //     uni_draw_rect(x0, y0, x1, y1, color);  // faster
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
                    float c_alpha = 1.0f, l_alpha = 0, r_alpha = 0;
                    if (dx)
                    {
                        float abs_err = helper::abs(err);
                        float length_f = length;
                        // Quantum mechanics here?
                        // the more accurate of what the color it is,
                        // the less accurate of where it is
                        // auto base = color / length;
                        // unless you use float to preserve more information
                        // but it's slightly more expensive (~2%)
                        auto base = 1.0f / length_f;
                        c_alpha = base * (length_f - abs_err);
                        abs_err = helper::abs(err - abs_dy);
                        abs_err = std::min(abs_err, length_f);
                        l_alpha = base * (length_f - abs_err);
                        abs_err = helper::abs(err + abs_dy);
                        abs_err = std::min(abs_err, length_f);
                        r_alpha = base * (length_f - abs_err);
                        if (step < 0)
                            std::swap(l_alpha, r_alpha);
                    }
                    if (auto x=x0-1; x >= 0)
                    {
                        uni_set_pixel(x, y0, color * l_alpha + uni_get_pixel(x, y0) * (1-l_alpha));
                    }
                    if (auto x=x0+1; x < WIDTH)
                    {
                        uni_set_pixel(x, y0, color * r_alpha + uni_get_pixel(x, y0) * (1-r_alpha));
                    }
                    uni_set_pixel(x0, y0, color * c_alpha + uni_get_pixel(x0, y0) * (1-c_alpha));
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
                    float c_alpha = 1.0f, u_alpha = 0, d_alpha = 0;
                    if (dy)
                    {
                        float abs_err = helper::abs(err);
                        float length_f = length;
                        // auto base = color / length;
                        auto base = 1.0f / length_f;
                        c_alpha = base * (length_f - abs_err);
                        abs_err = helper::abs(err - abs_dx);
                        abs_err = std::min(abs_err, length_f);
                        u_alpha = base * (length_f - abs_err);
                        abs_err = helper::abs(err + abs_dx);
                        abs_err = std::min(abs_err, length_f);
                        d_alpha = base * (length_f - abs_err);
                        if (step < 0)
                            std::swap(u_alpha, d_alpha);
                    }
                    if (auto y=y0-1; y >= 0)
                    {
                        uni_set_pixel(x0, y, color * d_alpha + uni_get_pixel(x0, y) * (1-d_alpha));
                    }
                    if (auto y=y0+1; y < HEIGHT)
                    {
                        uni_set_pixel(x0, y, color * u_alpha + uni_get_pixel(x0, y) * (1-u_alpha));
                    }
                    uni_set_pixel(x0, y0, color * c_alpha + uni_get_pixel(x0, y0) * (1-c_alpha));
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
            TransformRef ref=TransformRef::Parent,
            uint64_t timestamp_ms=stm32::clock::get_systick_ms())
        {
            Context ctx{
                .ref = ref,
                .x = x_offset,
                .y = y_offset,
                .sx = x_scale,
                .sy = y_scale,
                .rot = rotation,
                .parent = nullptr,
                .timestamp_ms = timestamp_ms,
                .canvas = *this
            };
            for (auto &child : children)
                child->draw(ctx);
        }

        constexpr virtual uni_t uni_get_pixel(coord_t x, coord_t y) const = 0;
        constexpr virtual void uni_set_pixel(coord_t x, coord_t y, uni_t color) = 0;

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
            coord_t x0, coord_t y0, coord_t x1, coord_t y1, uni_t color=PX_MAX,
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
                    inter_sects[1][0], inter_sects[1][1], color);
            
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
                        inter_sects[0][0], inter_sects[0][1], color);
                }
                // intersection[1] inside of the ray
                return _draw_seg(
                    x0, y0,
                    inter_sects[1][0], inter_sects[1][1], color);
            }
            // origin of the ray is outside
            return _draw_seg(
                inter_sects[0][0], inter_sects[0][1],
                inter_sects[1][0], inter_sects[1][1], color);

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
            coord_t x0, coord_t y0, coord_t x1, coord_t y1, uni_t color=PX_MAX)
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
                    uni_set_pixel(x, y1, color);
                }
            if (has_b)
                for (coord_t x=x0; x <= x1; ++x)
                {
                    uni_set_pixel(x, y0, color);
                }
            if (has_l)
                for (coord_t y=y0; y <= y1; ++y)
                {
                    uni_set_pixel(x0, y, color);
                }
            if (has_r)
                for (coord_t y=y0; y <= y1; ++y)
                {
                    uni_set_pixel(x1, y, color);
                }
        }
        constexpr virtual void uni_draw_filled_rect(
            coord_t x0, coord_t y0, coord_t x1, coord_t y1, uni_t color=PX_MAX)
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
                    uni_set_pixel(x, y, color);
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
            coord_t x0, coord_t y0, coord_t r, uni_t color=PX_MAX)
        {
            const auto WIDTH = get_width(), HEIGHT = get_height();
            r = helper::abs(r);
            if (x0 < -r or x0 > WIDTH + r or y0 < -r or y0 > HEIGHT + r)
                return;
            coord_t x = r, y = 0;
            coord_t err = 1-x;
            while (x >= y)
            {
                uni_set_pixel(x0 + x, y0 + y, color);
                uni_set_pixel(x0 + y, y0 + x, color);
                uni_set_pixel(x0 - y, y0 + x, color);
                uni_set_pixel(x0 - x, y0 + y, color);
                uni_set_pixel(x0 - x, y0 - y, color);
                uni_set_pixel(x0 - y, y0 - x, color);
                uni_set_pixel(x0 + y, y0 - x, color);
                uni_set_pixel(x0 + x, y0 - y, color);
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


    /**
     * @brief Canvas with fixed size
     * 
     * @tparam W Width
     * @tparam H Height
     * @tparam B Bits per pixel
     */
    template <size_t W, size_t H, uint8_t B, typename PX_T=uni_t>
    requires (B != 0 and B <= sizeof(uni_t) * 8)
    class Canvas : public BaseCanvas
    {
        using Mat_t = std::array<std::array<PX_T, W>, H>;
    public:
        Mat_t pixels{};
        constexpr coord_t get_width() const override { return W; }
        constexpr coord_t get_height() const override { return H; }
        constexpr uint8_t get_pixel_bits() const override { return B; }

        constexpr uni_t uni_get_pixel(const coord_t x, const coord_t y) const override
        {
            if (x < 0 or x >= static_cast<coord_t>(W) or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            auto pixel = pixels[y][x];
            return rescale_color(pixel, B);
        }
        constexpr void uni_set_pixel(
            const coord_t x, const coord_t y, const uni_t color=PX_MAX) override
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            pixels[y][x] = color >> (sizeof(uni_t) * 8 - B);  // rescale to pixel range
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

    template <size_t W, size_t H, uint8_t B>
    requires (B != 0 and B <= 4)
    class Canvas4B : public BaseCanvas
    {
        using Mat_t = std::array<std::array<uint8_t, W>, (H+1)/2>;
    public:
        Mat_t pixels{};
        constexpr coord_t get_width() const override { return W; }
        constexpr coord_t get_height() const override { return H; }
        constexpr uint8_t get_pixel_bits() const override { return B; }

        constexpr uni_t uni_get_pixel(
            const coord_t x, const coord_t y) const override
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            uni_t pixel = pixels[y/2][x];
            pixel = (y % 2) ? (pixel & 0x0F) : (pixel >> 4);
            return rescale_color(pixel, B);
        }
        constexpr void uni_set_pixel(
            const coord_t x, const coord_t y, const uni_t color=PX_MAX) override
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            uni_t pixel = color >> (sizeof(uni_t) * 8 - B);  // rescale to pixel range
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

    template <size_t W, size_t H, uint8_t B>
    requires (B != 0 and B <= 2)
    class Canvas2B : public BaseCanvas
    {
        using Mat_t = std::array<std::array<uint8_t, W>, (H+1)/4>;
    public:
        Mat_t pixels{};
        constexpr coord_t get_width() const override { return W; }
        constexpr coord_t get_height() const override { return H; }
        constexpr uint8_t get_pixel_bits() const override { return B; }

        constexpr uni_t uni_get_pixel(const coord_t x, const coord_t y) const override
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            uni_t pixel = pixels[y/4][x];
            const unsigned shift = (y % 4) * 2;
            pixel = (pixel >> shift) & 0x03;
            return rescale_color(pixel, B);
        }
        constexpr void uni_set_pixel(
            const coord_t x, const coord_t y, const uni_t color=PX_MAX) override
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            uni_t pixel = color >> (sizeof(uni_t) * 8 - B);  // rescale to pixel range
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

    template <size_t W, size_t H, uint8_t B=1>
    requires (B == 1)
    class Canvas1B : public BaseCanvas
    {
        using Mat_t = std::array<std::array<uint8_t, W>, (H+1)/8>;
    public:
        Mat_t pixels{};
        constexpr coord_t get_width() const override { return W; }
        constexpr coord_t get_height() const override { return H; }
        constexpr uint8_t get_pixel_bits() const override { return B; }

        constexpr uni_t uni_get_pixel(const coord_t x, const coord_t y) const override
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            uni_t pixel = pixels[y/8][x];
            const unsigned shift = y % 8;
            pixel = (pixel >> shift) & 0x01;
            return rescale_color(pixel, B);
        }
        constexpr void uni_set_pixel(
            const coord_t x, const coord_t y, const uni_t color=PX_MAX) override
        {
            if (x < 0 or x >= static_cast<coord_t>(W)
                or y < 0 or y >= static_cast<coord_t>(H))
                throw std::invalid_argument("coordinates out of range");
            uni_t pixel = color >> (sizeof(uni_t) * 8 - B);  // rescale to pixel range
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
                ctx.x + _x1, ctx.y + _y1, color, type);
        }
    public:
        coord_t x0, y0, x1, y1;
        uni_t color;
        LineType type;
    };

    class TextNode : public Node
    {
    protected:
        constexpr TextNode() : Node() {};
        void _draw_this_impl(const Context &ctx) override;
    public:
        std::string content{};
    };

}
}
