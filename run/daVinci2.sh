echo "Sourcing jacky_venv..."
source ~/Envs/jacky_venv/bin/activate
echo "Done!"

echo "Sourcing ros..."
source ~/Workspace/jacky_working/ros_ws/devel/setup.sh
roscd masters_control
echo "Done!"
