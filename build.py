import shutil, errno, os

# you need change this to your real port
com = "COM8"

def copyanything(src, dst):
    try:
        shutil.copytree(src, dst)
    except OSError as exc: # python >2.5
        if exc.errno in (errno.ENOTDIR, errno.EINVAL):
            shutil.copy(src, dst)
        else: raise

source_src = "./src/test"
source_des = "./kendryte-standalone-sdk/src/test"
if (os.path.exists(source_des)):
    shutil.rmtree("./kendryte-standalone-sdk/src/test")
copyanything("./src/test", "./kendryte-standalone-sdk/src/test/")

os.system('cmake --no-warn-unused-cli -DPROJ=test -DCMAKE_BUILD_TYPE:STRING=Debug -S ./kendryte-standalone-sdk -B ./build -G "MinGW Makefiles"')
os.system('cmake --build build')
os.system('kflash -p ' + com + ' -b 2000000 -f 0 --verbose --noansi --terminal ./build/test.bin')
