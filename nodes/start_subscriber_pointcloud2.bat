call c:\ros2\setup.bat
call ..\..\rmw_ws\install\setup.bat

set RMW_IMPLEMENTATION=rmw_ecal_dynamic_cpp

build\subscriber_pointcloud2\Release\subscriber_pointcloud2.exe
