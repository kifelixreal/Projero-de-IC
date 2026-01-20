from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'alexx_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kifelix',
    maintainer_email='kifelixreal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'rota1_teste = alexx_py.rota1_teste:main',  # Adicionado aqui: nome_executavel = pacote.modulo:função
		'rota1_teste_odom = alexx_py.rota1_teste_odom:main',  # Adicionado aqui: nome_executavel = pacote.modulo:função
		'rota1_teste_odom_2 = alexx_py.rota1_teste_odom_2:main',  # Adicionado aqui: nome_executavel = pacote.modulo:função
		'vision_node = alexx_py.vision_node:main',  # Ajuste o caminho se necessário
		'control_node = alexx_py.control_node:main',  # Controle 
		
        ],
    },
    
)
