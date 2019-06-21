from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    name='esiaf_pocketsphinx',
    version='0.0.1',
    description='A pocketsphinx node for the esiaf framework',
    url='---none---',
    author='rfeldhans',
    author_email='rfeldh@gmail.com',
    license='---none---',
    install_requires=[
        'pocketsphinx>=0.1.15',
    ],
    packages=['esiaf_pocketsphinx']

)

setup(**setup_args)