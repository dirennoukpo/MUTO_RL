"""
Documentation FR: src/muto_bringup/setup.py
Role: module du pipeline MUTO RS (robotique temps reel ROS2).
Details: ce fichier participe a la chaine capteurs->observation->decision->commande.
Contraintes: garder la coherence QoS, les unites SI et la robustesse aux timeouts.
Maintenance: toute evolution doit conserver la compatibilite des topics, services et paramètres.
"""

from setuptools import find_packages, setup

package_name = 'muto_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/pi_full.launch.py',
            'launch/jetson_full.launch.py',
            'launch/all_robots.launch.py',
            'launch/debug.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/qos_config.yaml',
            'config/system_params.yaml',
        ]),
        ('share/' + package_name + '/config/urdf', [
            'config/urdf/muto_rs.urdf.xacro',
            'config/urdf/materials.xacro',
        ]),
        ('share/' + package_name + '/config/rviz', ['config/rviz/muto_rs_viz.rviz']),
        ('share/' + package_name + '/data', [
            'data/observation_spec_v3.json',
            'data/validate_sim_real.py',
            'data/model_card_v003.json',
        ]),
        ('share/' + package_name + '/test', [
            'test/checklist_terrain.md',
        ]),
        ('share/' + package_name + '/test/phase1_hardware_pi', [
            'test/phase1_hardware_pi/test_usb_bridge_topics.py',
            'test/phase1_hardware_pi/test_watchdog_timeouts.py',
            'test/phase1_hardware_pi/test_mode_transitions.py',
        ]),
        ('share/' + package_name + '/test/phase2_control_jetson', [
            'test/phase2_control_jetson/test_obs_builder_vector.py',
            'test/phase2_control_jetson/test_safety_filter.py',
        ]),
        ('share/' + package_name + '/test/phase3_perception', [
            'test/phase3_perception/test_perception_no_rl.py',
        ]),
        ('share/' + package_name + '/test/phase4_ai', [
            'test/phase4_ai/test_rl_pipeline_hz.py',
            'test/phase4_ai/test_dry_run_commands.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='diren.noukpo@epitech.eu',
    description='Orchestration launch/config systeme Muto RS pour Pi et Jetson.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
