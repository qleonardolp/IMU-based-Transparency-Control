/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <stdio.h>
#include <string.h>
#include <cmath>

#ifndef M_PI
  #define M_PI 3.141592653f
  #define RADS2RPM (30/M_PI) // rad/s to rpm 
  #define RPM2RADS (M_PI/30) // rpm to rad/s
#endif

#define LPF_CUTOFF 7


/// @file   LowPassFilter2p.h
/// @brief  A class to implement a second order low pass filter
/// @authors: Leonard Hall <LeonardTHall@gmail.com>, template implmentation: Daniel Frenzel <dgdanielf@gmail.com>
template <class T>
class DigitalBiquadFilter {
public:
    struct biquad_params {
        float cutoff_freq;
        float sample_freq;
        float a1;
        float a2;
        float b0;
        float b1;
        float b2;
    };
  
    DigitalBiquadFilter();

    T apply(const T &sample, const struct biquad_params &params);
    void reset();
    static void compute_params(float sample_freq, float cutoff_freq, biquad_params &ret);
    
private:
    T _delay_element_1;
    T _delay_element_2;
};

template <class T>
class LowPassFilter2p {
public:
    LowPassFilter2p();
    // constructor
    LowPassFilter2p(float sample_freq, float cutoff_freq);
    // change parameters
    void set_cutoff_frequency(float sample_freq, float cutoff_freq);
    // return the cutoff frequency
    float get_cutoff_freq(void) const;
    float get_sample_freq(void) const;
    T apply(const T &sample);
    void reset(void);

protected:
    struct DigitalBiquadFilter<T>::biquad_params _params;
    
private:
    DigitalBiquadFilter<T> _filter;
};

typedef LowPassFilter2p<float>    LowPassFilter2pFloat;

//////////////////////////////////////////\/////////\//
// Leonardo Felipe Lima Santos dos Santos /\     ////\/
// leonardo.felipe.santos@usp.br	_____ ___  ___  //|
// github/bitbucket qleonardolp /	| |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  	| |   \ \   |_|  /|
//\///////////////////////\// ////	\_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\////\//
