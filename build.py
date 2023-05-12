import shutil, errno, os

# you need change this to your real port
com = "COM7"

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

os.system('cmake ./kendryte-standalone-sdk -B build -DPROJ=test -G "MinGW Makefiles"')
os.system('cmake --build build')
os.system('kflash -p ' + com + ' -b 3000000 --verbose --noansi --terminal build/test.bin')
