SPQR Ball Perceptor
=================

### Introduction

SPQR Ball Perceptor is a software module for black and white ball detection developed by the SPQR to be used within the BHuman framework. This repo is a fork from the official [2016 B-Human code release](https://github.com/bhuman/BHumanCodeRelease).

SPQR Ball Perceptor is distributed under the GPL license for the parts developed by SPQR Team. This license does not apply to the parts of this distribution developed by others where the rights of the copyright owners remain. In particular, see the file "License.txt" for more details about the parts that have not been developed by SPQR Team.

### A Supervised Approach

The SPQR Ball detector is based on a supervised approach implemented in OpenCV. In particular, an LBP binary cascade classifier has been trained to detect the ball.
Details about how to generate the classifier are available in the tutorial ["How to Use OpenCV for Ball Detection -
RoboCup SPL Use Case"](http://profs.scienze.univr.it/~bloisi/tutorial/balldetection.html).

### Results

SPQR Ball Perceptor has been used by the SPQR Robot Soccer Team during the competitions of the Robocup German Open 2017.


