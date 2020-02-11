# DB-Cmd-Vels
This node sends wheel commands to the [Duckietown](https://www.duckietown.org/) Duckiebot.

Documentation for the Duckietown project can be found [here](https://docs.duckietown.org/daffy/).

---
## Note!
The encoder_control node in this package has been designed to work in conjunction with the [db-read-encoders](https://github.com/mech-4640/db-read-encoders) node, which **MUST** be running first.
To install db-read-encoders, please run in terminal:

	$ cd ~/catkin_ws/src
	$ git clone https://github.com/mech-4640/db-read-encoders

To run the db-read-encoders node, please run:

	$ cd ~/catkin_ws/src/db-read-encoders
	$ dts devel build -f --arch arm32v7 -H islduckieXX.local
	$ docker -H islduckieXX.local run -it --rm --privileged --net=host duckietown/db-read-encoders:master-arm32v7


Note: Please replace the XX in `islduckieXX.local` in the command above with your Duckiebot number, or you may substitute this with the your Duckiebot's [hostname](https://docs.duckietown.org/DT19/opmanual_duckiebot/out/setup_duckiebot.html).

---
	
## How to use it

### 1. Fork this repository (Encouraged)

Use the fork button in the top-right corner of the github page to fork this template repository.


### 2. Create a new repository (Encouraged)

Create a new repository on github.com while
specifying the newly forked template repository as
a template for your new repository.


### 3. Define dependencies (Optional)

List the dependencies in the files `dependencies-apt.txt` and
`dependencies-py.txt` (apt packages and pip packages respectively).


### 4. Place your code

Place your ROS packages in the directory `/packages` of
your new repository.

NOTE: Do not use absolute paths in your code,
the code you place under `/packages` will be copied to
a different location later.

For this part, the package `/packages/my_package` has already been created to house your code. For courses at [Dalhousie](https://www.dal.ca/faculty/engineering/mechanical.html) using this repository, you may need to modify the `launch.sh`, `/packages/my_package/src/encoder_control.py` or the `encoder_control.launch`.


### 5. Setup the launchfile

Change the file `launch.sh` in your repository to
launch your code.

---
## This code borrowed from the Duckietown ROS template
See [the template](https://github.com/duckietown/template-ros)
