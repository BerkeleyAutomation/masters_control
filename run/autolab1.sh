echo "Sourcing alan_jacky..."
source activate alan_jacky
echo "Done\!"

echo "Sourcing ros..."
source "~/Workspace/jacky_working/ros_ws/devel_setup.sh"
roscd masters_control
echo "Done!"

echo "Starting video service..."
python scripts/video_saving_service.py &
echo "Done!"

echo "Starting pose mapping service..."
python scripts/masters_yumi_connector.py &
echo "Done!"

echo "Serving pose mapping and video service indefinitely..."
wait 
