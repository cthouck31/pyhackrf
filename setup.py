import os
from setuptools import setup, Extension
from setuptools.command.install import install
from setuptools.command.develop import develop
import subprocess


__NAME__    = 'pyhackrf'
__VERSION__ = '1.0.1'
__DIR__     = os.path.dirname(os.path.abspath(__file__)).rstrip('/')
__REQS__    = []

__PKG_REQS__ =\
    [
        'build-essential',
        'cmake',
        'libusb-1.0-0-dev',
        'pkg-config',
        'libfftw3-dev'
    ]

__SCRIPTS__ =\
    [
        'apps/hackrf_clt'
    ]


class libhackrfInstall(install):
    def run(self):
        command = "apt-get install hackrf",
        process = subprocess.Popen(command, shell=True, cwd=__NAME__)
        process.wait()
        install.run(self)


class libhackrfDevelop(develop):
    def run(self):
        command = "git clone https://github.com/mossmann/hackrf %s" % (__DIR__+'/hackrf')
        process = subprocess.Popen(command, shell=True, cwd=__NAME__)
        process.wait()
        self.__compile_libhackrf()
        develop.run(self)

    def __compile_libhackrf(self):
        cloneDir = __DIR__+'/hackrf'
        if not os.path.exists(cloneDir):
            print "Failed to find \'hackrf\' GIT repo."
        else:
            print "Found GIT repo for \'libhackrf\'"
            command = "apt-get install %s" % (' '.join(__PKG_REQS__))
            process = subprocess.Popen(command, shell=True)
            process.wait()

            command = "mkdir -p %s" % (cloneDir+'/host/build')
            command += " && "
            command += "cd %s" % (cloneDir+'/host/build')
            command += " && "
            command += "cmake .."
            process = subprocess.Popen(command, shell=True)
            process.wait()

            command = "cd %s" % (cloneDir+'/host/build')
            command += " && "
            command += "make"
            command += " && "
            command += "sudo make install"
            command += " && "
            command += "sudo ldconfig"
            process = subprocess.Popen(command, shell=True)
            process.wait()


setup(
    name=__NAME__,
    version=__VERSION__,
    packages=[__NAME__],
    install_requires=__REQS__,
    scripts=__SCRIPTS__,
    cmdclass=\
        {
            'install': libhackrfInstall,
            'develop': libhackrfDevelop
        },
    include_package_data=True
)
