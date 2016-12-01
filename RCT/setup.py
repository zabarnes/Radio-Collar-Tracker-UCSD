from setuptools import setup

version = "1.3.12"

setup(name='RCT',
      version=version,
      zip_safe=True,
      description='Collar Tracker',
      long_description='''To Track those colls''',
      author='UCSD',
      license='GPLv3',
      packages=['RCT',
                'RCT.lib'],
      scripts=['RCT/mp_slipmap.py',
               'RCT/mp_tile.py'],
      package_data={'RCT':
                    ['RCT/data/*.jpg', 
                     'RCT/data/*.png']}
    )
