#!/usr/bin/env python
import rosgraph

def generate_dot_file(output_file):
    try:
        # Create the ROS master API instance
        master = rosgraph.Master('/rosnode')

        # Get system state (publishers, subscribers, and services)
        publishers, subscribers, services = master.getSystemState()
        topic_types = dict(master.getTopicTypes())

        # Create a DOT file format representation
        with open(output_file, 'w') as f:
            f.write('digraph G {\n')

            # Add topics as nodes
            for topic in topic_types.keys():
                f.write(f'"{topic}" [shape=ellipse];\n')

            # Add publishers to topics
            for topic, nodes in publishers:
                for node in nodes:
                    f.write(f'"{node}" -> "{topic}" [label="publishes"];\n')

            # Add subscribers to topics
            for topic, nodes in subscribers:
                for node in nodes:
                    f.write(f'"{topic}" -> "{node}" [label="subscribes"];\n')

            # Add services (optional, not visualized in rqt_graph)
            for service, nodes in services:
                for node in nodes:
                    f.write(f'"{node}" -> "{service}" [label="provides"];\n')

            f.write('}\n')

        print(f"DOT graph saved to {output_file}")
    except Exception as e:
        print(f"Error generating DOT file: {e}")

if __name__ == '__main__':
    output_file = 'rosgraph.dot'
    generate_dot_file(output_file)
