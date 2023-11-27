#pragma once

#include <cstdint>
#include <vector>
#include <algorithm>
#include <utility>

namespace vermils
{
namespace stm32
{
namespace clock
{
namespace rcc
{
class Solution;
}

namespace tree
{

using ratio_t = std::pair<uint32_t, uint32_t>;

enum class Preference
{
    None,
    Lower,
    Higher
};

inline constexpr bool uint32_compare(uint32_t a, uint32_t b, uint32_t accepted_err=0) noexcept
{
    if (a > b)
        return a - b <= accepted_err;
    else
        return b - a <= accepted_err;
}

class Node
{
public:
    std::vector<Node *> parents;
    std::vector<Node *> children;
    std::vector<ratio_t> ratios;  // must be sorted from lowest to highest
    uint32_t target_hz=0;
    uint8_t parent_index=0;
    uint8_t ratio_index=0;
    bool locked=false;  // if true, frequency must be exactly target_hz
protected:
    uint8_t _shadow_parent_index=parent_index;
    uint8_t _shadow_ratio_index=ratio_index;
public:
    constexpr Node() = default;
    virtual constexpr ~Node() = default;

    virtual constexpr Node *get_parent() const
    {
        if (parents.empty())
            return nullptr;
        return parents.at(parent_index);
    }

    virtual constexpr Node *get_root() const
    {
        if (auto parent = get_parent())
            return parent->get_root();
        return const_cast<Node *>(this);
    }

    virtual constexpr ratio_t get_ratio() const
    {
        if (ratios.empty())
            return {1, 1};
        return ratios.at(ratio_index);
    }

    virtual constexpr uint32_t get_hz() const
    {
        if (auto parent = get_parent())
        {
            auto ratio = get_ratio();
            return parent->get_hz() * ratio.first / ratio.second;
        }
        return target_hz;
    }
    virtual void apply_self()
    {
        _shadow_parent_index = parent_index;
        _shadow_ratio_index = ratio_index;
    }
    virtual void apply_children()
    {
        for (auto child : children)
        {
            child->apply_self();
            child->apply_children();
        }
    }
    virtual void apply_all()
    {
        apply_self();
        apply_children();
    }
    virtual constexpr bool validate_self(uint32_t accepted_err=0) const noexcept
    {
        if (!locked)
            return true;
        return uint32_compare(get_hz(), target_hz, accepted_err);
    }
    virtual constexpr bool validate_children(uint32_t accepted_err=0) const noexcept
    {
        for (auto child : children)
        {
            if (!child->validate_self(accepted_err))
                return false;
            if (!child->validate_children(accepted_err))
                return false;
        }
        return true;
    }
    virtual constexpr bool validate_all(uint32_t accepted_err=0) const noexcept
    {
        if (!validate_self(accepted_err))
            return false;
        if (!validate_children(accepted_err))
            return false;
        return true;
    }
    virtual constexpr void reset_self() noexcept
    {
        parent_index = _shadow_parent_index;
        ratio_index = _shadow_ratio_index;
    }
    virtual constexpr void reset_children() noexcept
    {
        for (auto child : children)
        {
            child->reset_self();
            child->reset_children();
        }
    }
    virtual constexpr void reset_all() noexcept
    {
        reset_self();
        reset_children();
    }
};

}
}
}
}