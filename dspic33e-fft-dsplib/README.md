![image](../images/microchip.jpg)

##  FFT Implementation using DSP Library

## Description:

Microchip's 16-bit dsPIC® Digital Signal Controllers feature a DSP Engine in the CPU that is capable of executing a Fast Fourier Transform (FFT) 
with great efficiency (high speed and low RAM usage). The on-chip features enabling the FFT implementation include, bit-reversed addressing,
Multiply-accumulate (MAC) type instructions and the ability to store and retrieve constants stored in Program memory.

Microchip provides a DSP functions library that provides in-place FFT functions.

This code example demonstrates how the DSP library functions can be used to perform an FFT operation on an input signal (vector) and find the spectral component with the highest energy. 

## Configuration

### Reconfiguring the project for a different dsPIC33E device:

 The Project can be easily reconfigured for dspic33ep512gm710/dspic33ep512mu810/dspic33ep256gp506 device by following the below steps - 
1. Open project in MPLAB-X.
2. In MPLAB X>>Configuration drop-down option>>Listed Device Configuration
3. Re-build the MPLAB® project using the menu option: Clean and Build Main Project

### Reconfiguring the project for a different FFT Size:

The code example is reconfigurable to perfrom an FFT of any size, including common sizes of 64, 128, 256 and 512 points. The code example also allows the user to place 
the FFT coefficients (known as Twiddle Factors) in RAM or in Program Flash Memory. The project may be easily reconfigured by modifying the header
 file, [fft.h](firmware/src/fft.h).
- Change `FFT_BLOCK_LENGTH` to either 64, 128, 256 or 512
- Correspondingly, change `LOG2_BLOCK_LENGTH` to either 6, 7, 8 or 9 respectively.
- If you would like to store Twiddle Factors coefficients in RAM instead of Program Memory comment out the line - `#define FFTTWIDCOEFFS_IN_PROGMEM`

## Operation

The FFT is performed in the following steps:

1. Initialization: 

   i. Generate Twiddle Factor Coefficients and store them in X-RAM. The twiddle factors can be generated using the following function:
      ```C
      #ifndef FFTTWIDCOEFFS_IN_PROGMEM					/* Generate TwiddleFactor Coefficients */
           TwidFactorInit (LOG2_BLOCK_LENGTH, &twiddleFactors[0], 0);	/* We need to do this only once at start-up */
      #endif
      ```

      Alternatively, you may also use the pre-generated twiddle factors stored in Program Flash.

   ii. Initialize the input vector in Y-memory at an address aligned to `4 x FFT_BLOCK_LENGTH` as shown below.
      
      ```C
      fractcomplex sigCmpx[FFT_BLOCK_LENGTH] __attribute__ ((eds, space(ymemory), aligned (FFT_BLOCK_LENGTH * 2 *2)));
      ```
      
   iii. Scale the input signal to lie within the range _[-0.5, +0.5]_. For fixed point fractional input data, this translates to input samples in 
   the range _[0xC000,0x3FFF]_. The scaling is achieved by simply right-shifting the input samples by 1 bit, assuming the input samples lie 
   in the fixed point range _[0x8000,0x7FFF]_ or _[-1,+1)_.

   iv. Convert the real input signal vector to a complex vector by placing zeros in every other location to signify a complex input whose
 imaginary part is _0x0000_.


2. Butterfly computation: 
   
   This is achieved by performing a call to the FFTComplexIP() function.
   ```C
   #ifndef FFTTWIDCOEFFS_IN_PROGMEM
       FFTComplexIP (LOG2_BLOCK_LENGTH, &sigCmpx[0], &twiddleFactors[0], COEFFS_IN_DATA);
   #else
       FFTComplexIP (LOG2_BLOCK_LENGTH, &sigCmpx[0], (fractcomplex *) __builtin_psvoffset(&twiddleFactors[0]), (int) __builtin_psvpage(&twiddleFactors[0]));
   #endif
   ```


3. Bit-Reversed Re-ordering: The output array is re-ordered to be in bit-reversed order of the addresses. This is achieved by -
   ```C
   /* Store output samples in bit-reversed order of their addresses */
    BitReverseComplex (LOG2_BLOCK_LENGTH, &sigCmpx[0]);
   ```


4. SquareMagnitude computation: We then need to compute the magnitude of each complex element in the output vector, so that we can estimate 
the energy in each spectral component/frequency bin. This is achieved by a call to a DSP Library's routine, SquareMagnitudeCplx(). 
   ```C
   /* Compute the square magnitude of the complex FFT output array so we have a Real output vetor */
       SquareMagnitudeCplx(FFT_BLOCK_LENGTH/2, &sigCmpx[0], output);
   ```


5. Peak-picking: We then find the frequency component with the largest energy by using the VectorMax() routine in the DSP library.
   ```C
   /* Find the frequency Bin ( = index into the SigCmpx[] array) that has the largest energy i.e., the largest spectral component */
       VectorMax(FFT_BLOCK_LENGTH/2, output, &peakFrequencyBin);
   ```


6. Frequency Calculation: The value of the spectral component with the highest energy, in Hz, is calculated by multiplying the array index of 
the largest element in the output array with the spectral (bin) resolution as follows - 
   ```C
   /* Compute the frequency (in Hz) of the largest spectral component */
       peakFrequency = (uint32_t) peakFrequencyBin*((float)SAMPLING_RATE/FFT_BLOCK_LENGTH);
   ```


### Input

The input signal used in the example will be 512 points of a Square wave signal of frequency 1KHz sampled at 10 KHz.

### Output
The FFT operation is performed on the input signal, in-place. This means that the output of the FFT resides in the same RAM locations where the
 input signal used to reside. 

Observe output variable `peakFrequency` in debug mode. Value should be almost near to 1Khz



## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)
	
	
## Software Used 

- MPLAB® X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v2.00 or newer (https://www.microchip.com/xc)
- MPLAB® XC16/XC-DSC DSP Library. Documentation can be found at `compiler_installation_folder/docs/dsp_lib`