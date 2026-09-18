# NASA TM-86040 figure gallery

Frank W. Burcham, Jr. and Edward A. Haering, Jr., *Highly Integrated Digital Engine Control System on an F-15 Airplane*, NASA Technical Memorandum 86040, June 1984. The report declares the paper a U.S. government work in the public domain. The PNGs reproduce the published figures and captions from NTRS document 19840016519. PDF page numbers include the two cover pages. [NASA Technical Reports Server](https://ntrs.nasa.gov/citations/19840016519)

| Figure | Printed page / PDF page | Type | Content |
| --- | --- | --- | --- |
| [1](figure-01.png) | 6 / 8 | Diagram | HIDEC research aircraft equipment |
| [2](figure-02.png) | 6 / 8 | Block diagram | HIDEC hardware and data buses |
| [3](figure-03.png) | 7 / 9 | Cutaway | F100 EMD (PW-1128) components and augmentor |
| [4](figure-04.png) | 7 / 9 | Block diagram | Adaptive stall-margin mode |
| [5](figure-05.png) | 7 / 9 | Line plot | Inlet distortion versus angle of attack and sideslip at Mach 1.6 |
| [6](figure-06.png) | 8 / 10 | Stacked bars | Two fan stall-margin budgets |
| [7](figure-07.png) | 8 / 10 | Steady-state plots | EPR uptrim at intermediate power, low speed, and low altitude |
| [8](figure-08.png) | 8 / 10 | Steady-state plots | EPR uptrim at maximum power and subsonic high altitude |
| [9](figure-09.png) | 9 / 11 | Time-series plots | EMD linear-model response to a nozzle-driven uptrim at intermediate power |
| [10](figure-10.png) | 9 / 11 | Line plot | INTERACT study uptrim values at maximum power |
| [11](figure-11.png) | 10 / 12 | Concept diagram and plots | Flightpath management concepts |
| [12](figure-12.png) | 10 / 12 | Contour plot | Aircraft specific energy and specific excess power contours with minimum-time paths |

Figures 7 and 8 give approximate nozzle-area, fan stall-margin, and net-thrust curves versus EPR for separate steady-state F100 EMD model cases. Figure 9 gives approximate EPR, fan stall margin, thrust, nozzle area, and gas-generator fuel flow versus time from a separate linear model. The 20 °C scale in each FTIT panel supports temperature differences, while the axes leave the absolute baseline unspecified. The report states that the Figure 9 simulation omitted DEEC response. These figures record historical model results for the EMD, and the report does not supply the component maps or complete boundary conditions needed to reproduce them. Keep the EMD results distinct from the PW-100 flight comparison. [NASA TM-86040, printed pp. 3–4 and 8–9](https://ntrs.nasa.gov/citations/19840016519) · [PW-100 public comparison](../../../docs/pw100-public-comparison.md)

## Readings from Figures 7 and 8

Each row samples the interior of the historical steady-state model curves at a selected EPR. The readings use the visible axis ticks of each panel. Estimated scan-reading tolerances are ±0.01 EPR, ±0.02 ft², ±1 percentage point of stall margin, and ±50 lbf. The tolerances describe digitization. The report does not give corresponding test or model uncertainty for these curves.

| Figure | EPR | Nozzle area (ft²) | Fan stall margin (%) | Net thrust (lbf) |
| --- | ---: | ---: | ---: | ---: |
| 7 | 3.10 | 2.95 | 24.3 | 12,810 |
| 7 | 3.15 | 2.91 | 23.0 | 12,970 |
| 7 | 3.20 | 2.88 | 21.9 | 13,130 |
| 7 | 3.30 | 2.82 | 19.6 | 13,440 |
| 7 | 3.40 | 2.74 | 16.7 | 13,740 |
| 8 | 3.35 | 4.54 | 20.7 | 4,070 |
| 8 | 3.45 | 4.44 | 17.9 | 4,150 |
| 8 | 3.55 | 4.37 | 15.9 | 4,280 |
| 8 | 3.65 | 4.33 | 13.3 | 4,410 |
| 8 | 3.75 | 4.26 | 8.4 | 4,520 |

## Reproduce the crops

Use Poppler's `pdftoppm` with the original NTRS PDF. Run this from the repository root and set `pdf` to the local file. The crop boxes below use pixel coordinates on a page scaled to 1,300 pixels along its longest edge. The command renders each region at 1.5 times that scale.

```sh
pdf=/path/to/19840016519.pdf
out=notebooks/references/hidec
mkdir -p "$out"
while read -r number page x y right bottom; do
    pdftoppm -f "$page" -l "$page" -singlefile -scale-to 1950 -gray -png \
        -x "$((3 * x / 2))" -y "$((3 * y / 2))" \
        -W "$((3 * right / 2 - 3 * x / 2))" -H "$((3 * bottom / 2 - 3 * y / 2))" \
        "$pdf" "$out/figure-$number"
done <<'CROPS'
01 8 203 178 791 530
02 8 194 550 795 1048
03 9 225 108 766 402
04 9 97 430 912 898
05 9 336 925 664 1210
06 10 285 98 688 532
07 10 106 552 447 1225
08 10 548 559 838 1225
09 11 300 149 697 718
10 11 175 747 818 1138
11 12 142 140 838 701
12 12 299 727 686 1085
CROPS
```
