# dSPACE Maps for Scenic

This directory contains OpenDRIVE maps and reference data for the dSPACE simulator.

## Laguna Seca Racing Circuit

Generated from dSPACE RoadNetwork (.rd) file using the `rd_to_xodr.py` conversion tool.

### Files:

- **`Laguna_Seca.xodr`** - Main OpenDRIVE file with multi-road junction connections
- **`Laguna_Seca_The_Corkscrew.csv`** - Reference line data for the Corkscrew section (2484.5m)
- **`Laguna_Seca_Track_Section_2.csv`** - Reference line data for Track Section 2 (883.4m)  
- **`Laguna_Seca_Track_Section_3.csv`** - Reference line data for Track Section 3 (988.0m)

### Track Structure:

The Laguna Seca circuit consists of:
- **3 main road segments** representing different sections of the track
- **2 multi-road junctions** connecting the segments at natural convergence points
- **6 junction connector roads** bridging gaps between main segments
- **Total circuit length**: 4,356.0m

### Junction Layout:

- **Junction 1** (at -98.0, -480.9): Connects Corkscrew start + Section 3 end + Section 2 end
- **Junction 2** (at 182.6, 27.7): Connects Corkscrew end + Section 3 start + Section 2 start

### Usage:

Use `Laguna_Seca.xodr` as the map file for dSPACE simulations. The circuit supports:
- Continuous vehicle flow around the complete track
- Proper lane connections at junctions
- Counter-clockwise racing direction

Generated: $(Get-Date)
Conversion tool: `src/scenic/simulators/dspace/rd_to_xodr.py`
