
Radar data preprocessing block - xWRDataPreProc
=======================================================

## Overview

This repository contains preprocessing block used inside radar dsp chain. It do following:

* Pack raw radar data to specific form taking into account type of radar data (complex data or real data) - input is 16 bits data, output is 32 bits data that can be connected to the input of the fft block
* Does zero padding logic if that is necessary
* Generate last signal after each frame
* In the future, this block should take care of multi-chirp mode

**Note**  Project is work in process and it should be run with chipyard dependencies. Repository `dsputils` should be available as well inside chipyard repository.
