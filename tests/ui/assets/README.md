# SYDNEY digit font source

`BarlowCondensed-SemiBold.ttf` is the vector source used by `../generate_sydney_font.py`.
It is rasterized at four times the target resolution before conversion to the firmware's 4-bpp digit subset.
The firmware builds only the generated `application/ui_sydney_font.c`; it does not load the TTF at runtime.

Upstream: https://github.com/google/fonts/tree/main/ofl/barlowcondensed

Copyright 2017 The Barlow Project Authors. License: SIL Open Font License 1.1, included in `OFL.txt`.
