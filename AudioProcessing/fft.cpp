/* * Free Fast Fourier Transform (FFT)
 * Copyright (c) 2021 Nayuki. (MIT License)
 * https://www.nayuki.io/page/free-small-fft-in-multiple-languages
 */

 #include <algorithm>
 #define M_PI 3.14159265358979323846
 #include <cmath>
 #include <stdexcept>
 #include "fft.hpp"
 
 using std::complex;
 using std::size_t;
 using std::vector;
 
 
 void Fft::transform(vector<complex<double>> &vec) {
     size_t n = vec.size();
     if (n == 0)
         return;
     else if ((n & (n - 1)) == 0)  // Is power of 2
         transformRadix2(vec);
     else  // More complicated algorithm for arbitrary sizes
         throw std::domain_error("FFT size must be a power of 2 for this implementation.");
 }
 
 
 void Fft::inverseTransform(vector<complex<double>> &vec) {
     std::for_each(vec.begin(), vec.end(), [](complex<double> &c){ c = std::conj(c); });
     transform(vec);
     std::for_each(vec.begin(), vec.end(), [](complex<double> &c){ c = std::conj(c); });
 }
 
 
 void Fft::transformRadix2(vector<complex<double>> &vec) {
     // Length variables
     size_t n = vec.size();
     int levels = 0;
     for (size_t temp = n; temp > 1U; temp >>= 1)
         levels++;
     if (static_cast<size_t>(1U) << levels != n)
         throw std::domain_error("Length is not a power of 2");
     
     // Trignometric tables
     vector<complex<double>> expTable(n / 2);
     for (size_t i = 0; i < n / 2; i++)
         expTable[i] = std::exp(complex<double>(0, -2 * M_PI * i / n));
     
     // Bit-reversed addressing permutation
     for (size_t i = 0; i < n; i++) {
         size_t j = 0;
         for (size_t temp = i, k = 0; k < levels; k++, temp >>= 1)
             j = (j << 1) | (temp & 1U);
         if (j > i)
             std::swap(vec[i], vec[j]);
     }
     
     // Cooley-Tukey decimation-in-time radix-2 FFT
     for (size_t size = 2; size <= n; size *= 2) {
         size_t halfsize = size / 2;
         size_t tablestep = n / size;
         for (size_t i = 0; i < n; i += size) {
             for (size_t j = 0; j < halfsize; j++) {
                 size_t k = j * tablestep;
                 complex<double> temp = vec[i + j + halfsize] * expTable[k];
                 vec[i + j + halfsize] = vec[i + j] - temp;
                 vec[i + j] += temp;
             }
         }
     }
 }
 