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