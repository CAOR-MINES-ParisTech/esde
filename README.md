# Exploiting Symmetries to Design EKFs with Consistency Properties
### Martin Brossard, Axel Barrau and Silvère Bonnabel

This repo containt the Matlab code for reproducing the results of the paper **Exploiting Symmetries to Design EKFs with Consistency Properties for Navigation and SLAM**. Please follow the links to read the [paper](https://hal.archives-ouvertes.fr/hal-01874587/document).

## Simulation Results
The Matlab code of the Section **Simulation Results** is available in the folder ```simulations```.  The code is based on the two following references.

 - Axel Barrau and Silvère Bonnabel. **An EKF-SLAM algorithm with consistency properties**, April 2015.  [Arxiv preprint](https://arxiv.org/abs/1510.06263)
 - Guoquan Huang, Anastasios I. Mourikis and Stergios I. Roumeliotis, **Observability-based Rules for Designing Consistent EKF SLAM Estimators**, The International Journal of Robotics Research, vol. 29, no. 5, pp. 502-528, April 2010. [IJRR Online](http://journals.sagepub.com/doi/abs/10.1177/0278364909353640)

> **Notes:** 
>  - The code is based on the OC-EKF Matlab code of Prof. Guoquang Huang
>  - You can add iSAM results after installing it by following instructions at: iSAM [website](https://people.csail.mit.edu/kaess/isam/doc/index.html)
>  - You can simulation parameters in the main script (`main.m`) as needed
>  - Run `main.m` for Monte-Carlo simulations and plots


## Experimental Results

The Matlab code for the Section **Experimental Results** is available in folder `experiments`.  The dataset is described in the following reference.

- Keith YK Leung, Yoni Halpern, Timothy D Barfoot, Hugh HT Liu,  **The UTIAS Multi-Robot Cooperative Localization and Mapping Dataset** The International Journal of Robotics Research, Vol. 30, No. 8, pp 969-974, March 2011 [IJRR Online](http://ijr.sagepub.com/content/30/8/969) [Website](http://asrl.utias.utoronto.ca/datasets/mrclam/)

> **Notes:** 
>  - We use the Matlab scripts tools that are provided for the convenience of UTIAS dataset users (`loadMRCLAMdataSet.m`, `sampleMRCLAMdataSet.m`, `animateMRCLAMdataSet.m`)
>  - You can dowload data at [http://asrl.utias.utoronto.ca/datasets/mrclam/](http://asrl.utias.utoronto.ca/datasets/mrclam/) and then extract data in the folder `experiments`
>  - Run `main.m`

## Citing the paper

If you find this code useful for your research, please consider citing the following paper:

	@Inproceedings{brossard2018exploiting,
	  Title          = {Exploiting Symmetries to Design EKFs with Consistency Properties for Navigation and SLAM},
	  Author         = {Brossard, Martin and Barrau, Axel, and Bonnabel Silvère},
	  Year           = {2018},
	  Journal        = {IEEE Sensors Journal},
	}

##  License
For academic usage, the code is released under the permissive MIT license.

## Acknowledgements
We thank Guoquan Paul Huang of University of Delaware  for sharing his code of OC-EKF.

