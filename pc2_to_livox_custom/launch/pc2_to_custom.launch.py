import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 定义格式转换节点
    pc2_converter_node = Node(
        package='pc2_to_livox_custom',
        executable='pc2_to_custom_node',
        name='pc2_to_custom_node',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
        # 拓展性配置：通过重映射机制，解耦代码与具体的话题名称
        # 如果未来你的机器人换了名字，或者话题变了，只需要改这里，不需要重新编译 C++ 代码
        # remappings=[
        #     ('/red_standard_robot1/livox/lidar', '/your_new_input_topic'),
        #     ('/red_standard_robot1/livox/custom_msg', '/your_new_output_topic')
        # ]
    )

    return LaunchDescription([
        pc2_converter_node
    ])