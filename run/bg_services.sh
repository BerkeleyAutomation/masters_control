echo "Starting video service..."
python scripts/video_saving_service.py &
echo "Done!"

echo "Starting pose mapping service..."
python scripts/masters_yumi_connector.py &
echo "Done!"

echo "Serving pose mapping and video service indefinitely..."
wait 