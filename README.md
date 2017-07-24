SPQR Ball Perceptor
=================

***SPQR Ball Perceptor is provided WITHOUT ANY WARRANTY about its usability. It is for educational purposes and should be regarded as such. Please report bugs at sapienza.spqrteam@gmail.com***

### Introduction

SPQR Ball Perceptor is a software module for black and white ball detection developed by the [SPQR Team](http://spqr.diag.uniroma1.it/) to be used within the BHuman framework. This repo is a fork from the official [2016 B-Human code release](https://github.com/bhuman/BHumanCodeRelease).

SPQR Ball Perceptor is distributed under the LGPL-3.0 license for the parts developed by the [SPQR Team](http://spqr.diag.uniroma1.it/). This license does not apply to the parts of this distribution developed by others where the rights of the copyright owners remain. In particular, see the file "License.txt" for more details about the parts that have not been developed by the [SPQR Team](http://spqr.diag.uniroma1.it/).

The details about the SPQR Ball Perceptor can be found in the technical report:<br>
Domenico Bloisi, Francesco Del Duchetto, Tiziano Manoni, and Vincenzo Suriani
"Machine Learning for Realistic Ball Detection in RoboCup SPL"
[PDF](MachineLearningForRealisticBallDetection.pdf)


### A Supervised Approach

The SPQR Ball detector is based on a supervised approach implemented in OpenCV. In particular, an LBP binary cascade classifier has been trained to detect the ball.
Details about how to generate the classifier are available in the tutorial ["How to Use OpenCV for Ball Detection -
RoboCup SPL Use Case"](http://profs.scienze.univr.it/~bloisi/tutorial/balldetection.html).

### Installation

The SPQR Ball Perceptor has been tested on Ubuntu 16.04 LTS. The installation procedure is as follows.
1. Install dependencies:
  * cmake
  * libgtk2.0-dev
  * pkg-config
  * libavcodec-dev
  * libavformat-dev
  * libswscale-dev
  * libjpeg8
  * libjpeg-turbo8-dbg
  * lib32z1-dev
  * libjpeg-turbo8-dev

* To install the above dependencies on Ubuntu 16.04 use the following command:
`sudo apt−get install cmake libgtk2.0−dev pkg−config libavcodec−dev libavformat−dev libswscale−dev libjpeg8
libjpeg-turbo8−dbg lib32z1−dev libjpeg−turbo8−dev`

2. Install *Opencv 2.4*

3. By following the B-Human code release documentation it is possible to conclude the code installation. 

IMPORTANT: To improve the detection and to not discard far away balls it is necessary to calibrate the NAO's camera matrix and joints.


### Results

SPQR Ball Perceptor has been used by the [SPQR Team](http://spqr.diag.uniroma1.it/) during the competitions of the [Robocup German Open 2017](https://www.robocupgermanopen.de/en).

[![SPQR Ball Perceptor](http://img.youtube.com/vi/KFxiFpezvw0/0.jpg)](http://www.youtube.com/watch?v=KFxiFpezvw0 "SPQR Ball Perceptor")

SPQR Ball Perceptor can be used both indoor and outdoor without any modification.

[![SPQR Ball Perceptor indoor and outdoor](http://img.youtube.com/vi/fIgEwHRe6Bk/0.jpg)](http://www.youtube.com/watch?v=fIgEwHRe6Bk "SPQR Ball Perceptor indoor and outdoor")

### SPQR Team 2017

_Faculty_  
Daniele Nardi  
Luca Iocchi  

_Team Manager_  
Domenico Bloisi  

_Software Development Leader_  
Vincenzo Suriani  

_Members_  
Emanuele Antonioni  
Matteo Cecchini  
Francesco Del Duchetto  
Tiziano Manoni  
Armando Nania  
Marco Paolelli  
