/********************************************************************************
 * MIT License
 *
 * Copyright (c) 2020 Stereolabs
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ********************************************************************************/

#pragma once

#include <rclcpp/clock.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sl/Camera.hpp>
#include <string>
#include <chrono>

namespace zed_acquisition
{
/*!
 * \brief The CSmartMean class is used to
 * make a mobile window mean of a sequence of values
 * and reject outliers.
 * Tutorial:
 * https://www.myzhar.com/blog/tutorials/tutorial-exponential-weighted-average-good-moving-windows-average/
 */
class SmartMean
{
public:
  explicit SmartMean(int winSize);

  int getValCount()
  {
    return mValCount;  ///< Return the number of values in the sequence
  }

  double getMean()
  {
    return mMean;  ///< Return the updated mean
  }

  /*!
   * \brief addValue
   * Add a value to the sequence
   * \param val value to be added
   * \return mean value
   */
  double addValue(double val);

private:
  int mWinSize;   ///< The size of the window (number of values ti evaluate)
  int mValCount;  ///< The number of values in sequence

  double mMeanCorr;  ///< Used for bias correction
  double mMean;      ///< The mean of the last \ref mWinSize values

  double mGamma;  ///< Weight value
};

/**
 * @brief Stop Timer used to measure time intervals
 *
 */
class StopWatch
{
public:
  StopWatch();

  void tic();    //!< Set the beginning time
  double toc();  //!< Returns the seconds elapsed from the last tic

private:
  std::chrono::steady_clock::time_point mStartTime;
};

SmartMean::SmartMean(int winSize)
{
  mValCount = 0;

  mMeanCorr = 0.0;
  mMean = 0.0;
  mWinSize = winSize;

  mGamma = (static_cast<double>(mWinSize) - 1.) / static_cast<double>(mWinSize);
}

double SmartMean::addValue(double val)
{
  mValCount++;

  mMeanCorr = mGamma * mMeanCorr + (1. - mGamma) * val;
  mMean = mMeanCorr / (1. - pow(mGamma, mValCount));

  return mMean;
}

StopWatch::StopWatch()
{
  tic();
}

void StopWatch::tic()
{
  mStartTime = std::chrono::steady_clock::now();
}

double StopWatch::toc()
{
  auto now = std::chrono::steady_clock::now();
  double elapsed_usec =
    std::chrono::duration_cast<std::chrono::microseconds>(now - mStartTime).count();
  return elapsed_usec / 1e6;
}

}  // namespace zed_acquisition