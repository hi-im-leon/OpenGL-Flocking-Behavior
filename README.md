Implemented a flocking and seeking behavior for 20 individuals. The leader is a certain color (randomly generated) and all other followers are another:

To run the code:
<ol>
<li>Create a build file and run cmake ..</li>
<li>Make the files</li>
<li>Run ../bin/ChaseViewer</li>
</ol>

Some of the challenges Michael and I faced while making this file is adjusting the velocities and finding a proper way to ensure that they do not accrue or go overboard. While creating flocking behavior, we combine three behaviors: aligning, cohesion, and separation. Finding the appropriate balance is a bit of guess and check.
