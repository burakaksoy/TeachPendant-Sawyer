from setuptools import setup, find_packages, find_namespace_packages

setup(
    name='rr-plugins',
    version='0.0.1',
    description='PyRI Teach Pendant Plugins',
    author='Burak Aksoy',
    author_email='burakaksoy20@gmail.com',
    url='https://github.com/burakaksoy/TeachPendant-Sawyer/',
    package_dir={'': 'src'},
    packages=find_namespace_packages(where='src'),
    include_package_data=True,
    zip_safe=False,
    install_requires=[
    ],
    tests_require=['pytest','pytest-asyncio'],
    extras_require={
        'test': ['pytest','pytest-asyncio']
    },
    entry_points = {
        'console_scripts': ['pyri-core=pyri.core.core:main']
    }
)
