LocalFolder=/home/$USER/workspace/catkin_ws/src/mtracker/
RemoteFolder=/home/mtracker/catkin_ws/src/mtracker/

# TODO: check if folder exist
echo "---------------------------";
echo " Removing previous version ";
echo "---------------------------";
ssh mtracker@MTracker "rm -r $RemoteFolder"
ssh mtracker@MTracker "rm -r /home/mtracker/catkin_ws/build/mtracker/"
ssh mtracker@MTracker "rm -r /home/mtracker/catkin_ws/devel/share/mtracker/"
ssh mtracker@MTracker "rm -r /home/mtracker/catkin_ws/devel/lib/mtracker/"

echo "---------------------";
echo " Copying new version ";
echo "---------------------";
ssh mtracker@MTracker "mkdir $RemoteFolder"

## Do not copy git repository
scp -r ${LocalFolder}src/ mtracker@MTracker:${RemoteFolder}src/
scp -r ${LocalFolder}launch/ mtracker@MTracker:${RemoteFolder}launch/
scp -r ${LocalFolder}include/ mtracker@MTracker:${RemoteFolder}include/
scp -r ${LocalFolder}scripts/ mtracker@MTracker:${RemoteFolder}scripts/
scp -r ${LocalFolder}resources/ mtracker@MTracker:${RemoteFolder}resources/

scp ${LocalFolder}CMakeLists.txt mtracker@MTracker:${RemoteFolder}CMakeLists.txt
scp ${LocalFolder}package.xml mtracker@MTracker:${RemoteFolder}package.xml

echo "----------------------";
echo " Building new version ";
echo "----------------------";
ssh mtracker@MTracker "source /opt/ros/hydro/setup.bash && cd /home/mtracker/catkin_ws/ && catkin_make --pkg mtracker"

