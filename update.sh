cp ~/catkin_ws/src/roboquest_ui/src/settings.json ~/settings.backup.json
cd ~/catkin_ws/src/roboquest_ui/src
git fetch
git reset --hard HEAD
git merge '@{u}'
npm i
cp ~/settings.backup.json ~/catkin_ws/src/roboquest_ui/src/settings.json
