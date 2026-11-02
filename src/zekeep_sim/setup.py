from setuptools import find_packages, setup
setup(name='zekeep_sim',version='0.1.0',packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages',['resource/zekeep_sim']),
                  ('share/zekeep_sim',['package.xml','README.md'])],
      install_requires=['setuptools','numpy','mujoco'],zip_safe=True,
      maintainer='Zekeep Maintainers',maintainer_email='support@example.com',
      description='Isolated Six-axis MuJoCo physics and virtual RGB-D camera',license='Apache-2.0',
      entry_points={'console_scripts':['web_sim = zekeep_sim.server:main']})
