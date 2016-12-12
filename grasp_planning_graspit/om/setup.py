// setup.py
#!/usr/bin/env python
 
from distutils.core import setup
from distutils.extension import Extension
 
setup(name="PackageName",
    ext_modules=[
        Extension("autograsp_planning", ["autograsp_planning.cpp"],
        libraries = ["boost_python"])
    ])