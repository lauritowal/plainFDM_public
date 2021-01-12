from setuptools import setup, find_packages

setup(name='plain_fdm',
      version='0.0.1',
      packages=find_packages(),
      install_requires=['gym', 'pandas', 'matplotlib', 'pyglet', 'bokeh', 'pandas_bokeh'] # And any other dependencies required
)