# Adaptive Shielding with SUMO

This is part of the work for "Adaptive Shielding under Uncertainty" paper: https://adaptiveshielding.xyz/


## Install/Dependencies 

The SUMO framework provides the simulation platform for this project.
You find an install tutorial in the following link:
https://www.eclipse.org/sumo/

Please install the model checker STORM. 
You find an install tutorial in the following link:
https://www.stormchecker.org/getting-started.html

To build the code, we used CMake (Version 3.18.2).

## Build 

```
git clone https://github.com/shieldai/AdaptiveShielding.git
cd AdaptiveShielding
cmake .
make
```

## Test

The test data is recorded with SUMO 1.7.0 and STORM 1.6.2. 
A different version can lead to a different result and could end in failing tests.
Therefore, we recommend the Docker image ```mdei/shieldone:v1```, which is also set in the scripts.

By running the run script with the test argument, all basic test will be executed and compared to the test data:
```
./run.sh test
# OR
./run_docker.sh
```

## Run


```
Allowed options:
  -c [ --sumo ] arg            SUMO config file.
  -p [ --port ] arg            Port number for TraCI.
  -s [ --shield ] arg          Shield config files.
  -o [ --out ] arg             Outfile name for shielding log.
  -w [ --whitelist-tls ] arg   File with junction or tls IDs which will be 
                               shielded.
  -b [ --blacklist-tls ] arg   File with junction or tls IDs which will be not 
                               shielded.
  -i [ --incident-events ] arg File with traffic indecent event config.
  -u [ --update-interval ] arg Update interval of shields.
  -l [ --lambda ] arg          Learning rate (Shield Parameter lambda).
  -d [ --param-d ] arg         Shield Parameter d.
  -k [ --max-lane-size ] arg   Limit of recognized waiting vehicles on lanes 
                               (Shield Parameter K).
  -t [ --simulation-time ] arg Time step until stop the simulation.
  -x [ --warm-up-time ] arg    Time step until the shield starts to intervene.
  -g [ --gui ]                 Use sumo-gui.
  -f [ --free ]                Run without Shields.
  --bus                        Prioritize public transport.
  --no-lane-merging            Avoid the merging of parallel lanes.
  --static-update              Do shield updates in static interval no Minimum 
                               change for update required.
  --no-lane-trees              Disable accurate lane state for non OSM Maps.
  --side-by-side               Run a shielded and unshielded simulation 
                               simulations.
  --hook-sumo                  Connect to external started SUMO.
  --overwrite-controller       Overwrite the traffic light controller (RL 
                               Agent) with the shield strategy and reset to 
                               previous action if the overwritten controller 
                               takes not the control back.
  --help                       Help message.


./adaptiveShielding -c data/exp_basic/one_junction.sumo.cfg -g -d 4 -l 0.3 -k 10 -t 5000 -o log/demo.log
```

Please execute the SUMO scenarios with the shieldIDs.txt files and -w argument.

In the BASH file ``run.sh`` you find the right arguments to run the experiments.


## Run with Docker

The experiments can be executed with docker on Linux and windows and also support the GUI mode.
It requires an installed docker, and for windows, additionally, an [X11 server](https://sourceforge.net/projects/xming/) to support the GUI mode.

```
# ON LINUX
./run_docker.sh
# ON WINDOWS
./run_docker.bat 
```
