LocalFolder=/home/$USER/workspace/catkin_ws/src/mtracker/
RemoteFolder=/home/mtracker/catkin_ws/src/mtracker/

# Remove old version
ssh mtracker@MTracker "rm -r $RemoteFolder"
ssh mtracker@MTracker "mkdir $RemoteFolder && mkdir ${RemoteFolder}include/"
ssh mtracker@MTracker "rm -r /home/mtracker/catkin_ws/build/mtracker/"
ssh mtracker@MTracker "rm -r /home/mtracker/catkin_ws/devel/include/mtracker/"
ssh mtracker@MTracker "rm -r /home/mtracker/catkin_ws/devel/share/mtracker/"
ssh mtracker@MTracker "rm -r /home/mtracker/catkin_ws/devel/lib/mtracker/"

## Copy new version
scp -r ${LocalFolder}src/ mtracker@MTracker:${RemoteFolder}src/
scp -r ${LocalFolder}launch/ mtracker@MTracker:${RemoteFolder}launch/
scp -r ${LocalFolder}include/ mtracker@MTracker:${RemoteFolder}include/mtracker/
scp -r ${LocalFolder}scripts/ mtracker@MTracker:${RemoteFolder}scripts/
scp -r ${LocalFolder}resources/ mtracker@MTracker:${RemoteFolder}resources/

scp ${LocalFolder}CMakeLists.txt mtracker@MTracker:${RemoteFolder}CMakeLists.txt
scp ${LocalFolder}package.xml mtracker@MTracker:${RemoteFolder}package.xml

# Build new version
ssh mtracker@MTracker "source /opt/ros/hydro/setup.bash && cd /home/mtracker/catkin_ws/ && catkin_make --pkg mtracker"

