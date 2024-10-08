from launch import LaunchDescription                  
from launch.actions import DeclareLaunchArgument      
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node                   


def generate_launch_description():                    
   rboot_enable_status_launch_arg = DeclareLaunchArgument(
      # enable rboot acuators controller status reports every 10ms
      'enable_status', default_value=TextSubstitution(text='0')    
   )

   return LaunchDescription([                                     
      rboot_enable_status_launch_arg,                                    
      Node(                                                        
         package='rboot_topic',
         executable='topic_helloworld_sub',                              
         name='rboot',                                              
         parameters=[{                                            
            'enable_status': LaunchConfiguration('enable_status'), 
         }]
      ),
   ])
