
<br />

<h3 align="center">LiDART Board Control</h3>

  <p align="center">
    A list of scripts that give the user the necessary documents and instructions to control LiDART's Board, along with theorical simulations and a camera tester.
    
  </p>
</div>


<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
  </ol>
</details>

## Getting Started

All scripts included in this repository were used with Python 3.9.13 in a virtual environment managed by Python's own 'venv'. The following instructions will cover all dependencies along with some extra configurations needed to make things work.

Instructions will be for Windows 10 or 11, replication for Ubuntu or any other Linux distribution shall be adjusted accordingly.

### Prerequisites

To start, make sure Python 3.9.13 is installed (Usage of other versions might have unintended side effects, but possibly shouldn't interfere much). For this, you can go to [Python's downloads page](https://www.python.org/downloads/) and download the correct version. Also check [Python's documentation](https://wiki.python.org/moin/BeginnersGuide/Download) if unsure how to install.

This repository will not cover how to setup a virtual environment (venv), but the user is recommended to create one and use it for this project.

It is also needed to install the camera's drivers to be properly recognized by the python scripts, [IDS Software Suite](https://en.ids-imaging.com/ids-software-suite.html). Any further information can be found [here](https://www.1stvision.com/cameras/IDS/IDS-manuals/uEye_Manual/hw_grundlagen_schnellstart.html).

### Installation

Once Python is installed, there's a few libraries that need to be installed. For that we can use the ``requirements.txt`` provided. 
Open a terminal on the projects folder and use the following command:

```bash
pip install -r requirements.txt
```

Now with the libraries installed, we need to modify a file in order to allow the scripts to run. Where python is installed navigate to ``\Lib\site-packages\simple_pyueye``, there we'll need to modify or in this case, replace the file named ``camera.py`` for the one inside the folder ``simple_pyueye`` of this repository. 

The file changes the functions ``capture_still`` and ``continuous_capture``, so if you want, you can simply copy-paste both functions from one file to the other.
<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Usage

We can break down this repository into 3 parts:

- Practical:
	- ``boardControl.py``: For this we need both the camera and the board connected. It's responsible for controlling the board so that pictures can be taken for posterior analysis.
- Theorical:
	- ``theoricalSimulation.py``: Theorical simulation of the laser's spot on the camera's sensor when the target is a white paper;
	- ``theoricalSim_2Lasers.py``: Theorical simulation of two separated laser's spot, on the camera's sensor when the target is a white paper;
- Demos and dependencies:
	- ``pixelClock.py``: Demo script used initially to get the minimum camera exposure time and for better understanding of camera tweaking;
	- ``pixelClock_range.py``: Like the previous one;
	- ``utils.py``: Needed file for extra functions to tweak the camera further;
	- ``SimpleLive_Pyueye_OpenCV.py``: Script to check if the camera is working properly and if it is being properly identified. Also used to see the laser and adjust the camera's focus and position before taking pictures with ``boardControl.py``.

<p align="right">(<a href="#readme-top">back to top</a>)</p>