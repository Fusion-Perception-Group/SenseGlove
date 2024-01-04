#pragma once

#include <cstdint>
#include <array>
#include "property.hpp"
#include "gpio.hpp"
#include "units.hpp"

namespace vermils
{
namespace gesture
{
    using namespace stm32;

    enum class FingerType : uint8_t
    {
        Thumb,
        Index,
        Middle,
        Ring,
        Pinky,
    };

    enum class FingerStatus : uint8_t
    {
        Unknown,
        Open,
        Relaxed,
        Closed,
    };

    class GestureError : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    class Uncalibrated : public GestureError
    {
    public:
        Uncalibrated(const char *msg="Uncalibrated: Gesture sensor is not calibrated") : GestureError(msg) {}
    };

    class UnboundFinger : public GestureError
    {
    public:
        UnboundFinger(const char *msg="UnboundFinger: Cannot get curvature") : GestureError(msg) {}
    };

    /**
     * @brief Finger abstraction
     * 
     * @details value of curvature represents the curvature of the finger
     */
    class Finger
    {
        mutable FingerStatus _previous = FingerStatus::Unknown;
    public:
        using curv_t = int16_t;
        static constexpr const curv_t UNSET_V = std::numeric_limits<curv_t>::min();
        struct Threshold
        {
            curv_t threshold=UNSET_V;
            curv_t deviation=UNSET_V;
        };
        FingerType type=FingerType::Thumb;
        volatile curv_t *curvature=nullptr;
        Threshold open_thre{};
        Threshold relax_thre{};
        Threshold close_thre{};

        /**
         * @brief Check if the sensor curvature is bound to an address
         * 
         */
        bool is_bound() const { return curvature; }

        /**
         * @brief Check if the finger is fully open
         * 
         * @throw UncalibratedError if the sensor is not calibrated
         */
        bool is_fully_open(const bool unbound_default=false) const
        {
            if (open_thre.threshold == UNSET_V)
                throw Uncalibrated("Finger curvature not calibrated");
            
            if (!is_bound())
                return unbound_default;
            
            if (
                // (*curvature >= open_thre.threshold - open_thre.deviation) &&  // kept for debug
                (*curvature <= open_thre.threshold + open_thre.deviation))
            {
                _previous = FingerStatus::Open;
                return true;
            }
            return false;
        }

        /**
         * @brief Check if the finger is open in a more forgiving way
         * 
         * @param unbound_default 
         * @throw UncalibratedError if the sensor is not calibrated
         */
        bool is_open(const bool unbound_default=false) const
        {
            if (is_fully_open(unbound_default))
                return true;
            
            if (_previous == FingerStatus::Open)
            {
                if (relax_thre.threshold == UNSET_V)
                    throw Uncalibrated("Finger curvature not calibrated");
                return (*curvature < relax_thre.threshold - relax_thre.deviation);
            }
            return false;
        }

        /**
         * @brief Check if the finger is fully relaxed
         * 
         * @throw UncalibratedError if the sensor is not calibrated
         */
        bool is_fully_relaxed(const bool unbound_default=false) const
        {
            if (relax_thre.threshold == UNSET_V)
                throw Uncalibrated("Finger curvature not calibrated");

            if (!is_bound())
                return unbound_default;
            
            if ((*curvature >= relax_thre.threshold - relax_thre.deviation) &&
                (*curvature <= relax_thre.threshold + relax_thre.deviation))
            {
                _previous = FingerStatus::Relaxed;
                return true;
            }
            return false;
        }

        /**
         * @brief Check if the finger is relaxed in a more forgiving way
         * 
         * @throw UncalibratedError if the sensor is not calibrated
         */
        bool is_relaxed(const bool unbound_default=false) const
        {
            if (is_fully_relaxed(unbound_default))
                return true;
            
            if (_previous == FingerStatus::Relaxed)
            {
                if (close_thre.threshold == UNSET_V or open_thre.threshold == UNSET_V)
                    throw Uncalibrated("Finger curvature not calibrated");
                return (
                    (*curvature < close_thre.threshold - close_thre.deviation) &&
                    (*curvature > open_thre.threshold + open_thre.deviation)
                );
            }
            return false;
        }

        /**
         * @brief Check if the finger is fully closed
         * 
         * @throw UncalibratedError if the sensor is not calibrated
         */
        bool is_fully_closed(const bool unbound_default=false) const
        {
            if (close_thre.threshold == UNSET_V)
                throw Uncalibrated("Finger curvature not calibrated");
            
            if (!is_bound())
                return unbound_default;
            
            if (// (*curvature <= close_thre.threshold + close_thre.deviation) &&  // kept for debug
                (*curvature >= close_thre.threshold - close_thre.deviation))
            {
                _previous = FingerStatus::Closed;
                return true;
            }
            return false;
        }

        /**
         * @brief Check if the finger is closed in a more forgiving way
         * 
         * @throw UncalibratedError if the sensor is not calibrated
         */
        bool is_closed(const bool unbound_default=false) const
        {
            if (is_fully_closed(unbound_default))
                return true;
            
            if (_previous == FingerStatus::Closed)
            {
                if (relax_thre.threshold == UNSET_V)
                    throw Uncalibrated("Finger curvature not calibrated");
                return (*curvature > relax_thre.threshold + relax_thre.deviation);
            }
            return false;
        }

        FingerStatus get_status() const
        {
            if (is_open())
                return FingerStatus::Open;
            if (is_relaxed())
                return FingerStatus::Relaxed;
            if (is_closed())
                return FingerStatus::Closed;
            return FingerStatus::Unknown;
        }
    };

    class Hand
    {
        template <typename T>
        struct _Property : public tricks::StaticReadOnlyProperty<T, Hand&>
        {
            constexpr _Property(Hand &owner) : tricks::StaticReadOnlyProperty<T, Hand&>(owner) {}
            using tricks::StaticReadOnlyProperty<T, Hand&>::operator=;
        };
        struct _Finger : public _Property<Finger&>
        {
            const uint8_t order;
            constexpr _Finger(Hand &owner, const uint8_t order) : _Property<Finger&>(owner), order(order) {}
            Finger &getter() const override { return owner.fingers[order]; }
        };
    public:
        static constexpr const uint8_t FINGER_CNT = 5;
        using FingerArray = std::array<Finger, FINGER_CNT>;
        FingerArray fingers{};
        _Finger thumb{*this, 0};
        _Finger index{*this, 1};
        _Finger middle{*this, 2};
        _Finger ring{*this, 3};
        _Finger pinky{*this, 4};

        Hand(
            Finger f1=Finger{},
            Finger f2=Finger{},
            Finger f3=Finger{},
            Finger f4=Finger{},
            Finger f5=Finger{}
        )
        {
            f1.type = FingerType::Thumb;
            fingers[0] = f1;
            f2.type = FingerType::Index;
            fingers[1] = f2;
            f3.type = FingerType::Middle;
            fingers[2] = f3;
            f4.type = FingerType::Ring;
            fingers[3] = f4;
            f5.type = FingerType::Pinky;
            fingers[4] = f5;
        }

        /**
         * @brief Check if the hand is open
         * 
         * @param threshold The number of fingers that must be open
         * @throw UncalibratedError if the sensor is not calibrated
         */
        bool is_open(uint8_t threshold=FINGER_CNT, const bool unbound_default=true) const
        {
            threshold = std::min<uint8_t>(threshold, FINGER_CNT);
            for (uint8_t i=FINGER_CNT; i--;)
            {
                if (fingers[i].is_open(unbound_default))
                    --threshold;
                if (!threshold)
                    return true;
            }
            return false;
        }

        /**
         * @brief Check if the hand is relaxed
         * 
         * @param threshold The number of fingers that must be relaxed
         * @throw UncalibratedError if the sensor is not calibrated
         */
        bool is_relaxed(uint8_t threshold=FINGER_CNT, const bool unbound_default=true) const
        {
            threshold = std::min<uint8_t>(threshold, FINGER_CNT);
            for (uint8_t i=FINGER_CNT; i--;)
            {
                if (fingers[i].is_relaxed(unbound_default))
                    --threshold;
                if (!threshold)
                    return true;
            }
            return false;
        }

        /**
         * @brief Check if the hand is closed
         * 
         * @param threshold The number of fingers that must be closed
         * @throw UncalibratedError if the sensor is not calibrated
         */
        bool is_closed(uint8_t threshold=FINGER_CNT, const bool unbound_default=true) const
        {
            threshold = std::min<uint8_t>(threshold, FINGER_CNT);
            for (uint8_t i=FINGER_CNT; i--;)
            {
                if (fingers[i].is_closed(unbound_default))
                    --threshold;
                if (!threshold)
                    return true;
            }
            return false;
        }

        /**
         * @brief pinting with certain finger, default to index
         * 
         * @param finger 
         * @throw UncalibratedError if the sensor is not calibrated
         * @throw UnboundFinger if the finger is not bound
         */
        bool is_pointing(const FingerType finger=FingerType::Index) const
        {
            auto &tgt_f = fingers[static_cast<uint8_t>(finger)];
            if (!tgt_f.is_bound())
                throw UnboundFinger();
            for (auto &f : fingers)
            {
                if (f.type == finger)
                    continue;
                if (f.is_open())
                    return false;
            }
            return tgt_f.is_open();
        }
        bool is_pointing(const Finger finger) const
        {
            return is_pointing(finger.type);
        }

        
    };
}
}