# Automatic shelf detection and decluttering in warehouse environments
## Introduction
This work is my masters thesis at Örebro University (January - June, 2022).

My masters thesis was, actually, comprised of two individual theses; decluttering of indoor environments and shelf detection in indoor warehouse environments. Due to the nature of shelf detection and benefits provided of having a decluttered map the two were made in unison. Something that, in the end, yielded the metrics
`Precision = Recall = F1 = 100%` for the automatic shelf detector proposed in the thesis.

## Decluttering
Decluttering was done by naively adapting two-dimensional ROSE by [Kucner et al.](https://arxiv.org/pdf/2004.08794.pdf) by implementing a variety of grid map generation methods.

## Automatic shelf detection
Automatic shelf detection was solved by using Principal Component Analysis to find neighborhoods in the environment that was vertical and line shaped enough to, possibly, be the supporting poles of shelves searched for.