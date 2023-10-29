/**
 * @file    fstring.hpp
 * @brief   A quick C++ formatting library
 * @version 0.1
 *
 * Copyright (c) 2022 Maysara Elshewehy (xeerx.com) (maysara.elshewehy@gmail.com)
 *
 * Distributed under the MIT License (MIT) 
*/

#pragma once
#include <cstddef>      // std::size_t
#include <cstdint>      // std::uint_fast16_t
#include <string>       // std::string
#include <stdexcept>    // std::runtime_error
#include <ctype.h>      // isdigit
#include <vector>       // std::vector
#include <algorithm>    // std::find

// to save padding begin,end positions and value
struct padd_vec
{
    std::size_t begin, end, value;
    padd_vec(std::size_t b,std::size_t e, std::size_t v) : begin{b}, end{e}, value{v} {}
};


class fstring
{
    private:
    std::string src;
    std::size_t lpos = 0; // last postion for variables() function
    std::vector<padd_vec> poss; // to remember positions before formatting in padding()

    protected:

    // find paddings in string and handle it
    void padding() noexcept(false) 
    {
        // init
        std::size_t pos = 0, end = 0;

        // handle
        while(pos < src.length())
        {
            // find '%'
            if((pos = src.find('%', (pos == 0) ? 0 : pos+1)) == std::string::npos) break;

            // now we got the position of first "%", so increase position to get next character
            std::size_t begin = pos;  // save begin position
            ++pos;

            // what if the character "%" is last character in the string ?
            if(pos >= src.length()) break;

            // find '.' : for smart padding (padding - length of section)
            bool smart_padding = src.at(pos) == '.';
            if(smart_padding) 
            {
                // increase position to get the next character after "%."
                ++pos;

                // what if the character '.' is last character in the string ?
                if(pos >= src.length()) break;
            }

            // find padding
            std::uint_fast16_t padd   = 0;    // to store padding value
            std::uint_fast16_t digits = 0;    // to store digits count

            if (pos < src.length() && isdigit(src.at(pos))) // check if digit is number
            {
                // get the number
                std::uint_fast16_t n = src.at(pos)-'0';

                // if the first number is zero
                if(!n && !padd) throw std::runtime_error("Padding can not be zero OR start with zero");

                // calc padding
                padd = 10 * padd + n;

                // increase position to get the next character
                ++pos;

                // increase count of digits
                ++digits;

                while(pos < src.length())
                {
                    // check if digit is number
                    if(!isdigit(src.at(pos))) break;

                    // get the number
                    std::uint_fast16_t n = src.at(pos)-'0';

                    // calc padding
                    padd = 10 * padd + n;

                    // increase position to get the next character
                    ++pos;

                    // increase count of digits
                    ++digits;
                }
            }


            // if padding is zero so there is nothing to do so skip it !
            if(!padd) continue;

            // smart padding: save positions to apply after putting the variables value
            if(smart_padding)
            {
                // find ".%" : the end of section
                if ((end = src.find(".%", pos)) == std::string::npos) throw std::runtime_error("smart padding must start by %. and ended by .%");
                poss.push_back(padd_vec(begin,end-4,padd));
                
                src.erase(end,2);        // erase ".%"
                src.erase(begin,digits+2); // erase "%.n"
            }
            // normal padding: apply
            else
            {
                // erase %n
                src.erase(begin,digits+1); 
                // add spaces(padding)
                src.insert(begin , std::string(padd, ' ')); 

                // update position to avoid searching in spaces area
                pos += padd - (digits+1);
            }
        }
    }

    // apply smart padding
    void apply  ()
    {
        for (auto i = poss.begin(); i < poss.end(); i++) 
        {
            // get length of section
            std::size_t length = i->end - i->begin;

            // if length < padding value so there are a space to add padding
            if(i->value > length)
            {
                // calc padding value: padding - length = the rest of space
                std::size_t padd = i->value - length;

                // add spaces(padding)
                src.insert(i->end, std::string(padd, ' '));

                // now we updated the string value, so the length is updated
                // so  we need to update the upcoming positions with: +=padd
                for (auto t = i; t < poss.end(); t++) { t->begin += padd; t->end   += padd; }
            }
        }
    }

    // inline helpers for variables()
    inline std::string tostr(std::string  ref) { return ref;                 }
    inline std::string tostr(const char * ref) { return std::string(ref);    }
    template<typename Arg>
    inline std::string tostr(Arg          ref) { return std::to_string(ref); }

    // replace "{}" by values
    template<typename T>
    void variables(T arg)
    {
        //  to avoid error(out of range exception)
        if(lpos == std::string::npos) return;

        // find "{}"
        if ((lpos = src.find("{}", lpos)) == std::string::npos) return;

        // store arg value as string
        std::string val = tostr(arg);

        // save
        src.erase(lpos,2);    // erase "{}"
        src.insert(lpos,val); // add arg value in position of "{}"

        // now we updated the string value, so the length is updated
        // so  we need to update the upcoming positions with: +=val.length()
        for (auto i = poss.begin(); i < poss.end(); i++) 
        {
            // skip values smaller than the lpos
            if(i->end <= lpos) continue;

            // increase
            i->end   += val.length();

            // we cleared the "{}" characters so we need to go back 2 characters if possible
            if(i->end   > 2) i->end   -=2;

            // begin of non-first
            if(i != poss.begin())
            {
                i->begin += val.length();
                if(i->begin > 2) i->begin -=2;
            }
        }
    }

    public:

    // constructor
    template<typename... T>
    fstring(std::string _src, T&& ...args) : src{_src}
    {
        // performance improvement, see: https://cplusplus.com/reference/vector/vector/reserve/
        poss.reserve(  5);
        src .reserve(250);

        padding();
        ( variables(args) , ...);
        apply  ();
    }
    
    // get result
    std::string& get(){ return src; } 
};