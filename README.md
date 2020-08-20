# Visualize Point Density

## Overview
Visualize point density of the point cloud.

## Usage
```
=================================
     Visualize Point Density
         Tomomasa Uchida
           2020/08/20
=================================

  USAGE:
  ./vpd [input.spbr] [sigma_section_for_outlier] [output.spbr]

  EXAMPLE:
  ./vpd input.ply 2 output.spbr

   [sigma_section_for_outlier]
    0: No Outlier
    1: 1σ < Outlier
    2: 2σ < Outlier
    3: 3σ < Outlier
```

### Octree
```
Set divide value. (search radius = diagonal length / divide value): 500
> search radius: 0.0211673 (= 10.5836/500)

Now Octree Searching... 
10000000, 408 (points)
20000000, 261 (points)
Done Octree Search! (10.421 [minute])

Max num of neighborhood points: 2272
Min num of neighborhood points: 1
Average: 375.53
Standard Deviation: 300.97

Removed outliers for point density vector.
New max num of points: 977

Normalized point density vector.
Max num of points: 1

Writing spbr file (SPBR_DATA/test.spbr)...
Done writing spbr file! (0.868 [minute])
```

### PCL_RadiusSearch
```
Select search type. (0: RadiusSearch or 1: NearestKSearch): 0
> RadiusSearch

Set divide value. (search_radius = diagonal_length/divide_value): 50000
> search_radius: 0.00349226 (= 174.613/50000)

Now searching and calculating ...
*** Num. of processed points: 1000000 [9.97391 %]
*** Num. of processed points: 2000000 [19.9478 %]
*** Num. of processed points: 3000000 [29.9217 %]
*** Num. of processed points: 4000000 [39.8956 %]
*** Num. of processed points: 5000000 [49.8695 %]
*** Num. of processed points: 6000000 [59.8434 %]
*** Num. of processed points: 7000000 [69.8173 %]
*** Num. of processed points: 8000000 [79.7913 %]
*** Num. of processed points: 9000000 [89.7652 %]
*** Num. of processed points: 10000000 [99.7391 %]
Done! 0.264192 (minute)

Max num of points: 179
Min num of points: 1
Average: 4.25974
SD: 11.9164

Adjusted point densities vector.
Max num of points: 16

Normalized point densities vector.
Max num of points: 1

Writing spbr file (SPBR_DATA/borobu_out.spbr)...
```


### PCL_NearestKSearch
```
Select search type.(0:RadiusSearch or 1:NeatestKSearch) : 1
> NearestKSearch

Set nearest K: 1000
> 1000

Clock start
time : 881.543

Max point density : 0.30644
Min point density : 0.016313
```

## Visualization Results
|L=1|L=50|L=100|
|:-:|:-:|:-:|
|<img src="figures/LiDAR/upper_L1.bmp" width="1000">|<img src="figures/LiDAR/upper_L50.bmp" width="1000">|<img src="figures/LiDAR/upper_L100.bmp" width="1000">|

|Ookabuto|Fune-hoko float|Borobudur|
|:-:|:-:|:-:|
|<img src="figures/ookabuto.png" width="1000">|<img src="figures/funehoko_L100.png" width="1000">|<img src="figures/nakajimake_L100.png" width="1100">|