import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
def generate_launch_description():
	pkgPath=launch_ros.substitutions.FindPackageShare(package='urdf_test').find('urdf_test')
	urdfModelPath=os.path.join(pkgPath, 'urdf/model.urdf')
	rvizConfigPath= os.path.join(pkgPath, 'config/config.rviz')
	
	print(urdfModelPath)
	with open(urdfModelPath,'r') as infp:
		robot_desc=infp.read()
	params={'robot_description': robot_desc}
		
	robot_state_publisher_node=launch_ros.actions.Node(package='robot_state_publisher',executable='robot_state_publisher',output='screen',parameters=[params],arguments=[urdfModelPath])
	
	
	
	rviz_node=launch_ros.actions.Node(package='rviz2',executable='rviz2',name='rviz2',output='screen',arguments=['-d',rvizConfigPath])
	
	return launch.LaunchDescription([robot_state_publisher_node,rviz_node])

