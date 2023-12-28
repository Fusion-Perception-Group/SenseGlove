#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <cstdint>

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
    }
    using coord_t = int32_t;
    using uni_t = uint32_t;

    class Node; // forward declaration
    class BaseCanvas; // forward declaration
    using NodePtr = std::shared_ptr<Node>;
    using ConstNodePtr = std::shared_ptr<const Node>;
    using NodeContainer = std::vector<NodePtr>;
    inline constexpr uni_t PX_MAX = std::numeric_limits<uni_t>::max();
    inline constexpr uni_t PX_MIN = std::numeric_limits<uni_t>::min();

    enum class LineType
    {
        Infinite,
        Segment,
        Ray
    };
    struct Context
    {
        coord_t x, y;
        NodePtr parent;
        size_t timestamp_ms;
        BaseCanvas &canvas;
    };

    template <typename T>
    constexpr NodePtr make_nodeptr() requires std::derived_from<T, Node>
    {
        return std::make_shared<T>();
    }

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

        void add_child(NodePtr child) { children.push_back(child); }
        void add_child(Node& child);

        size_t insert_child(NodePtr child, const size_t index)
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

        template <typename T>
        NodePtr create_child() requires std::derived_from<T, Node>;

        template <typename T>
        NodePtr create_child(size_t index) requires std::derived_from<T, Node>;
    };

    class Node : public BaseNode, protected std::enable_shared_from_this<Node>
    {
    protected:
        constexpr Node() {};
    public:
        bool enabled=true;
        coord_t x=0, y=0;
        NodePtr operator&() { return shared_from_this(); }
        ConstNodePtr operator&() const { return shared_from_this(); }


        virtual void draw_this(const Context &ctx) = 0;
        void draw(const Context &ctx)
        {
            if (not enabled)
                return;
            Context new_ctx = ctx;
            new_ctx.x += x;
            new_ctx.y += y;
            new_ctx.parent = operator&();
            draw_this(new_ctx);
            for (auto &child : children)
                child->draw(new_ctx);
        }
    };

    class BaseCanvas : public BaseNode
    {
        void _draw_line(
            coord_t x0, coord_t y0, coord_t x1, coord_t y1, const uni_t color)
        {
            ;
        }
    public:
        BaseCanvas() = default;

        constexpr virtual coord_t get_width() const = 0;
        constexpr virtual coord_t get_height() const = 0;
        constexpr virtual uint8_t get_pixel_bits() const = 0;

        constexpr virtual uni_t uni_get_pixel(coord_t x, coord_t y) const = 0;
        constexpr virtual void uni_set_pixel(coord_t x, coord_t y, uni_t color) = 0;

        constexpr virtual void uni_draw_line(
            coord_t x0, coord_t y0, coord_t x1, coord_t y1, uni_t color=PX_MAX,
            LineType type=LineType::Segment)
        {
            // const coord_t HEIGHT = get_height(), WIDTH = get_width();
            // coord_t dx = x1 - x0;
            // coord_t dy = y1 - y0;
            // unsigned inter_cnt = 0;
            // coord_t inter_sects[2][2];

            // if (not dx and not dy)
            // {
            //     if (dx >= 0 and dx < WIDTH and dy >= 0 and dy < HEIGHT)
            //         uni_set_pixel(x0, y0, color);
            //     return;  // prevent division by zero
            // }

            // if (not dx)  // prevent division by zero
            // {
            //     if (x0 >= 0 and x0 < WIDTH)  // vertical line
            //     {
            //         inter_cnt = 2;
            //         inter_sects[0][0] = x0;
            //         inter_sects[0][1] = 0;
            //         inter_sects[1][0] = x0;
            //         inter_sects[1][1] = HEIGHT - 1;
            //     }
            // }
            // else if (not dy)  // prevent division by zero
            // {
            //     if (y0 >= 0 and y0 < HEIGHT)  // horizontal line
            //     {
            //         inter_cnt = 2;
            //         inter_sects[0][0] = 0;
            //         inter_sects[0][1] = y0;
            //         inter_sects[1][0] = WIDTH - 1;
            //         inter_sects[1][1] = y0;
            //     }
            // }
            // else
            // {
            //     coord_t tmp;
            //     // intersection with left edge
            //     tmp = (-x0 * dy / dx) + y0;
            //     if (tmp >= 0 and tmp < HEIGHT)
            //     {
            //         inter_sects[inter_cnt][0] = 0;
            //         inter_sects[inter_cnt][1] = tmp;
            //         ++inter_cnt;
            //     }
            //     // intersection with right edge
            //     tmp = ((WIDTH - 1 - x0) * dy / dx) + y0;
            //     if (tmp >= 0 and tmp < HEIGHT)
            //     {
            //         inter_sects[inter_cnt][0] = WIDTH - 1;
            //         inter_sects[inter_cnt][1] = tmp;
            //         ++inter_cnt;
            //     }
            //     if (inter_cnt != 2)  // may have found two intersections
            //     {
            //         // intersection with top edge
            //         tmp = (-y0 * dx / dy) + x0;
            //         if (tmp >= 0 and tmp < WIDTH
            //             and not (inter_cnt  // prevent duplicate points
            //                     and inter_sects[0][0] == tmp
            //                     and inter_sects[0][1] == 0))
            //         {
            //             inter_sects[inter_cnt][0] = tmp;
            //             inter_sects[inter_cnt][1] = 0;
            //             ++inter_cnt;
            //         }
            //     }
            //     if (inter_cnt != 2)  // may have found two intersections
            //     {
            //         // intersection with bottom edge
            //         tmp = ((HEIGHT - 1 - y0) * dx / dy) + x0;
            //         if (tmp >= 0 and tmp < WIDTH)
            //         {
            //             inter_sects[inter_cnt][0] = tmp;
            //             inter_sects[inter_cnt][1] = HEIGHT - 1;
            //             ++inter_cnt;
            //         }
            //     }
            // }

            // if (not inter_cnt)
            //     return;  // line outside the canvas
            
            // if (type == LineType::Infinite)
            //     return _draw_line(
            //         inter_sects[0][0], inter_sects[0][1],
            //         inter_sects[1][0], inter_sects[1][1], color);
            
            // if (type == LineType::Ray)
            // {
            //     coord_t dix=0, diy=0;
            //     if (dx * ())
            // }

        }
        // constexpr virtual void uni_draw_line(
        //     coord_t x0, coord_t y0, coord_t x1, coord_t y1, uni_t color=PX_MAX,
        //     LineType type=LineType::Segment)
        // {
        //     const coord_t HEIGHT = get_height(), WIDTH = get_width();
        //     coord_t euc_dist = 0; // euclidean distance from pixel to line
        //     coord_t dx = x1 - x0;
        //     coord_t dy = y1 - y0;
        //     int err=0;
        //     if (not dx and not dy) // prevent division by zero
        //     {
        //         uni_set_pixel(x0, y0, color);
        //         return;
        //     }

        //     dy = (dy > 0) ? dy : -dy;
        //     if (dx < 0)  // make sure x0 < x1
        //     {
        //         dx = -dx;
        //         auto tmp = x0;
        //         x0 = x1;
        //         x1 = tmp;
        //         tmp = y0;
        //         y0 = y1;
        //         y1 = tmp;
        //     }

        //     if (x0 >= WIDTH or x1 < 0)
        //         return;  // line outside of canvas

        //     if (y0 >= HEIGHT)
        //     {
        //         if (dx * (HEIGHT - 1 - y0) < dy * (WIDTH - 1 - x0))
        //             return; // line outside the canvas
        //         y0 = HEIGHT - 1;
        //         x0 += dx * (HEIGHT - 1 - y0) / dy;
        //     } // (x0, y0) outside the canvas, remap to the edge of the canvas

        //     if (x1 >= WIDTH || y1 >= HEIGHT)
        //     {
        //         if (dy * (WIDTH - x0) + y0 >= HEIGHT * dx)
        //         {
        //             x1 = x0 + (HEIGHT - y0) * dx / dy;
        //             y1 = HEIGHT - 1;
        //         }
        //         else
        //         {
        //             y1 = y0 + (WIDTH - x0) * dy / dx;
        //             x1 = WIDTH - 1;
        //         } // (x1, y1) outside the canvas, remap to the edge of the canvas
        //     }

        //     const coord_t length = helper::int_sqrt64(dx*dx + dy*dy);
        //     const bool vscan = dy > dx;  // vertical scan?
            
        //     int step;
        //     if (vscan)
        //     {
        //         if (dy < 0)  // always start from bottom to top
        //         {
        //             auto tmp = y0;
        //             y0 = y1;
        //             y1 = tmp;
        //             tmp = x0;
        //             x0 = x1;
        //             x1 = tmp;
        //         }
        //     }
        // }
        constexpr virtual void uni_draw_rect(coord_t x, coord_t y, coord_t w, coord_t h, uni_t color=PX_MAX)
        {}
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
    template <typename T>
    inline NodePtr BaseNodecreate_child() requires std::derived_from<T, Node>
    {
        auto child = make_nodeptr<T>();
        add_child(child);
        return child;
    }
    template <typename T>
    inline NodePtr BaseNode::create_child(size_t index) requires std::derived_from<T, Node>
    {
        auto child = make_nodeptr<T>();
        insert_child(child, index);
        return child;
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
            if (static_cast<uni_t>(x) >= W or static_cast<uni_t>(y) >= H)
                throw std::invalid_argument("coordinates out of range");
            auto pixel = pixels[y][x];
            return rescale_color(pixel, B);
        }
        constexpr void uni_set_pixel(const coord_t x, const coord_t y, const uni_t color=PX_MAX) override
        {
            if (static_cast<uni_t>(x) >= W or static_cast<uni_t>(y) >= H)
                throw std::invalid_argument("coordinates out of range");
            pixels[y][x] = color >> (sizeof(uni_t) * 8 - B);  // rescale to pixel range
        }

        constexpr PX_T get_pixel(const coord_t x, const coord_t y) const
        {
            if (static_cast<uni_t>(x) >= W or static_cast<uni_t>(y) >= H)
                throw std::invalid_argument("coordinates out of range");
            return pixels[y][x];
        }
        constexpr void set_pixel(const coord_t x, const coord_t y, const PX_T color)
        {
            if (static_cast<uni_t>(x) >= W or static_cast<uni_t>(y) >= H)
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

        constexpr uni_t uni_get_pixel(const coord_t x, const coord_t y) const override
        {
            if (static_cast<uni_t>(x) >= W or static_cast<uni_t>(y) >= H)
                throw std::invalid_argument("coordinates out of range");
            uni_t pixel = pixels[y/2][x];
            pixel = (y % 2) ? (pixel & 0x0F) : (pixel >> 4);
            return rescale_color(pixel, B);
        }
        constexpr void uni_set_pixel(const coord_t x, const coord_t y, const uni_t color=PX_MAX) override
        {
            if (static_cast<uni_t>(x) >= W or static_cast<uni_t>(y) >= H)
                throw std::invalid_argument("coordinates out of range");
            uni_t pixel = color >> (sizeof(uni_t) * 8 - B);  // rescale to pixel range
            if (y % 2)
                pixels[y/2][x] = (pixels[y/2][x] & 0xF0) | pixel;
            else
                pixels[y/2][x] = (pixels[y/2][x] & 0x0F) | (pixel << 4);
        }

        constexpr uint8_t get_pixel(const coord_t x, const coord_t y) const
        {
            if (static_cast<uni_t>(x) >= W or static_cast<uni_t>(y) >= H)
                throw std::invalid_argument("coordinates out of range");
            return (y % 2) ? (pixels[y/2][x] & 0x0F) : (pixels[y/2][x] >> 4);
        }
        constexpr void set_pixel(const coord_t x, const coord_t y, const uint8_t color)
        {
            if (static_cast<uni_t>(x) >= W or static_cast<uni_t>(y) >= H)
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
            if (static_cast<uni_t>(x) >= W or static_cast<uni_t>(y) >= H)
                throw std::invalid_argument("coordinates out of range");
            uni_t pixel = pixels[y/4][x];
            const unsigned shift = (y % 4) * 2;
            pixel = (pixel >> shift) & 0x03;
            return rescale_color(pixel, B);
        }
        constexpr void uni_set_pixel(const coord_t x, const coord_t y, const uni_t color=PX_MAX) override
        {
            if (static_cast<uni_t>(x) >= W or static_cast<uni_t>(y) >= H)
                throw std::invalid_argument("coordinates out of range");
            uni_t pixel = color >> (sizeof(uni_t) * 8 - B);  // rescale to pixel range
            const unsigned shift = (y % 4) * 2;
            pixels[y/4][x] = (pixels[y/4][x] & ~(0x03 << shift)) | (pixel << shift);
        }

        constexpr uint8_t get_pixel(const coord_t x, const coord_t y) const
        {
            if (static_cast<uni_t>(x) >= W or static_cast<uni_t>(y) >= H)
                throw std::invalid_argument("coordinates out of range");
            const unsigned shift = (y % 4) * 2;
            return (pixels[y/4][x] >> shift) & 0x03;
        }
        constexpr void set_pixel(const coord_t x, const coord_t y, const uint8_t color)
        {
            if (static_cast<uni_t>(x) >= W or static_cast<uni_t>(y) >= H)
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
            if (static_cast<uni_t>(x) >= W or static_cast<uni_t>(y) >= H)
                throw std::invalid_argument("coordinates out of range");
            uni_t pixel = pixels[y/8][x];
            const unsigned shift = y % 8;
            pixel = (pixel >> shift) & 0x01;
            pixel <<= (sizeof(uni_t) * 8 - B);  // rescale to full range
            return rescale_color(pixel, B);
        }
        constexpr void uni_set_pixel(const coord_t x, const coord_t y, const uni_t color=PX_MAX) override
        {
            if (static_cast<uni_t>(x) >= W or static_cast<uni_t>(y) >= H)
                throw std::invalid_argument("coordinates out of range");
            uni_t pixel = color >> (sizeof(uni_t) * 8 - B);  // rescale to pixel range
            const unsigned shift = y % 8;
            pixels[y/8][x] = (pixels[y/8][x] & ~(0x01 << shift)) | (pixel << shift);
        }

        constexpr bool get_pixel(const coord_t x, const coord_t y) const
        {
            if (static_cast<uni_t>(x) >= W or static_cast<uni_t>(y) >= H)
                throw std::invalid_argument("coordinates out of range");
            const unsigned shift = y % 8;
            return (pixels[y/8][x] >> shift) & 0x01;
        }
        constexpr void set_pixel(const coord_t x, const coord_t y, const bool color)
        {
            if (static_cast<uni_t>(x) >= W or static_cast<uni_t>(y) >= H)
                throw std::invalid_argument("coordinates out of range");
            const unsigned shift = y % 8;
            pixels[y/8][x] = (pixels[y/8][x] & ~(0x01 << shift)) | (static_cast<uint8_t>(color) << shift);
        }
    };

}
}
