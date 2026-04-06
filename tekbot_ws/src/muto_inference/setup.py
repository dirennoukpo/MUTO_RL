"""
Documentation FR: src/muto_inference/setup.py
Role: module du pipeline MUTO RS (robotique temps reel ROS2).
Details: ce fichier participe a la chaine capteurs->observation->decision->commande.
Contraintes: garder la coherence QoS, les unites SI et la robustesse aux timeouts.
Maintenance: toute evolution doit conserver la compatibilite des topics, services et paramètres.
"""

from setuptools import find_packages, setup

package_name = 'muto_inference'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rl_policy.launch.py']),
        ('share/' + package_name + '/config', ['config/rl_policy_params.yaml']),
        ('share/' + package_name + '/models', ['models/model_card_v003.json', 'models/norm_stats_v3.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='diren.noukpo@epitech.eu',
    description='Noeud inference RL TensorRT avec dry-run et fallback securise.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rl_policy_node = muto_inference.rl_policy_node:main',
        ],
    },
)
