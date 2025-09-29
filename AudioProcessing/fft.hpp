/* * Free Fast Fourier Transform (FFT)
 * Copyright (c) 2021 Nayuki. (MIT License)
 * https://www.nayuki.io/page/free-small-fft-in-multiple-languages
 */

 #pragma once

 #include <vector>
 #define M_PI 3.14159265358979323846
 #include <complex>
 
 namespace Fft {
     /* * Computes the discrete Fourier transform (DFT) of the given complex vector, storing the result back into the vector.
      * The vector can have any length. This is a wrapper function.
      */
     void transform(std::vector<std::complex<double>> &vec);
 
     /* * Computes the inverse discrete Fourier transform (IDFT) of the given complex vector, storing the result back into the vector.
      * The vector can have any length. This is a wrapper function.
      * This transform does not perform scaling, so the inverse is not a true inverse.
      */
     void inverseTransform(std::vector<std::complex<double>> &vec);
 
     /* * Computes the discrete Fourier transform (DFT) of the given complex vector, storing the result back into the vector.
      * The vector's length must be a power of 2. Uses the Cooley-Tukey decimation-in-time radix-2 algorithm.
      */
     void transformRadix2(std::vector<std::complex<double>> &vec);
 }
 