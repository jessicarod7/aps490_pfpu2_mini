# Automated Brain Detection System

For the Multidisciplinary Capstone Project at the University of Toronto, our team (PFPU2) was tasked
with identifying methods to avoid unintentional brain damage during autopsies. Typically, this is
the result of the varying thickness of cranial bone, which makes it difficult to estimate the
distance to the brain. When combined with the rapid turnaround needed in pathology units, this means
some damage from the autopsy saw is not uncommon.

Our solution was a contact detection system, which measures conductivity by passing a high-frequency
AC signal through the saw blade, and measuring the output to identify fluctuations in the signal. It
is inspired and derived from the SawStop® automatic braking technology used in woodworking.

You can find a copy of our showcase poster [here](./Capstone_Poster_FINAL_-_cmyk_300_dpi.pdf).

## About the code

This pure Rust project was designed for the RP2040 (mainly the Pi Pico board). It generates a 100
kHz signal using PWM and measures the output with the ADC. The current implementation could use some
improvements in analyzing the signal (especially with timing), but was sufficient to identify
contact with highly conductive surfaces, and shows indications of doing the same with capacitance.

If you're building on the design, here are some useful references:

- The schematic for our design can be found in the hardware branch
  of [jessicarod7/aps490_retraction_fsm](https://github.com/jessicarod7/aps490_retraction_fsm/tree/hardware).
  Don't copy the battery charger, instead reference
  the [Pi Pico datasheet §4.5-4.6](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf#page-21).
- Look at the high/low voltage range of the signal, but also _the voltage itself_. I found it
  increases significantly with proximity (not just contact) with highly conductive surfaces, like
  the capacitive capabilities of the SawStop.
  See [the poster](./Capstone_Poster_FINAL_-_cmyk_300_dpi.pdf) (§5.0, _Detected ΔV<sub>avg</sub>_)
  for more information.

The [`logs/`](./logs) folder contains some recorded test data used in system validation. It's not
critical to the program.

## About us

We are undergraduate students completing our BASc:

- Zainab Ali, Mechanical Engineering
- Olivia Lotzer, Mechanical Engineering
- Jessica Rodriguez, Computer Engineering
- Tina Sokhanvar, Mechanical Engineering
- Zeynep Tibik, Mechanical Engineering

I'm Jessica, and wrote the software in this repo.

## Acknowledgements

We would like to thank the following individuals:

- Our client, Dr. Christoper Ball, forensic pathologist with the Provincial Forensic Pathology
  Unit (PFPU) in Ontario, Canada
- Our industry advisor, Dr. Fabio Tironi, forensic pathologist
- Our supervisor and APS490 course coordinator, Dr. Kamran Behdinan, professor in Mechanical
  Engineering and UT-IMDI director
- Our TA, Ranjan Mishra, PhD candidate

We would also like to thank our additional contact points, the PFPU, and the entire APS490 course
team for their assistance.

### SawStop

We took inspiration from the SawStop design, covered by the now-expired US patent 7,055,417, and
which may be covered by continuations or international patents. We built our proof-of-concept for
experimental use only, and have no plans to scale. If you intend to build on our design, you should
consult for legal advice, especially if you plan to do so at a larger scale or commercially.

SawStop and all related terms are trademarks or registered trademarks of SawStop, LLC.

## License

Copyright 2024 Jessica Rodriguez

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
