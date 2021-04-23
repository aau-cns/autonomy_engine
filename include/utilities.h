// Copyright (C) 2021 Christian Brommer and Alessandro Fornasier,
// Control of Networked Systems, Universitaet Klagenfurt, Austria
//
// You can contact the author at <christian.brommer@ieee.org>
// and <alessandro.fornasier@ieee.org>
//
// All rights reserved.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#ifndef UTILITIES_H
#define UTILITIES_H

#include <iterator>
#include <iostream>

/**
 * @brief Function to print a vector
 * @param output stream (std::cout usually)
 * @param vector to be printed (elements of vector must be printable)
 */
template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {

  // Check vector is not empty
  if (!v.empty())
  {
    // Copy element of vector into output stream
    std::copy (v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
  }
  return out;
}

#endif  // UTILITIES_H
