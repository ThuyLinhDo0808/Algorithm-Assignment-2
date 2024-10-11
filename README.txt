The main script is mazeTester2.py.
There are three sample configuration files.  These specify the parameters used to run assignment 2.

To run the script, go to the same folder/directory that this README.txt file is located, and execute:

On Windows:
    python mazeTester2.py sampleConfig01TaskA.json
On Unix, particularly core teaching servers:
    python3 mazeTester2.py sampleConfig01TaskA.json


On the core teaching servers, matplotlib isn't installed, hence visualisation won't work.  Please run the visualisation on your own machines.

Please avoid modifying the provided files, apart from implementing the specified scripts (check the first line in each .py file).
If you must modify anything, please check in with the teaching team first.

Please note that the main differences between the configuration files for Task A versus Task B and C are:

* For Task A, we need the "solverEntranceIndex" key since it will specify the index of the entrance that the solver needs to focus on. 
  For task B and C, the configuration file should not include any entry for "solverEntranceIndex".

** For task B and C,the number of entrances and exits must be the same and the program needs to focus on finding paths
   between (entrance[0], exit[0]), (entrance[1], exit[1])...(entrance[k], exit[k])