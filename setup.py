"""
Setup of YuMi Teleop
Author: Jacky Liang
"""
from setuptools import setup

setup(name='yumi_teleop',
      version='0.1.dev0',
      description='YuMi Python Teleop Interface with DVRK by Berkeley AutoLab',
      author='Jacky Liang',
      author_email='jackyliang@berkeley.edu',
      package_dir = {'': '.'},
      packages=['yumi_teleop'],
     )
