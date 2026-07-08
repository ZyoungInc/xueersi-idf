# main.py
# ESP32 小喵掌机硬件线索探查脚本
# 建议拔掉电机后运行。电机部分使用 FakeI2C 拦截，不会真实驱动电机。

import os
import sys

def section(title):
    print("\n" + "=" * 60)
    print(title)
    print("=" * 60)

def hx(x):
    return "0x%02X" % int(x)

def hexlist(data):
    try:
        return "[" + ", ".join([hx(x) for x in data]) + "]"
    except Exception:
        return str(data)

def safe(desc, fn):
    section(desc)
    try:
        return fn()
    except Exception as e:
        print("ERROR:", repr(e))
        return None

def list_dir(path):
    try:
        print(path + ":", os.listdir(path))
    except Exception as e:
        print(path + ":", "ERROR", repr(e))

def try_read_source(name):
    section("尝试读取源码: " + name)
    paths = [
        name + ".py",
        "/" + name + ".py",
        "/app/" + name + ".py",
        "/cfg/" + name + ".py",
        "/lib/" + name + ".py",
    ]
    found = False
    for p in paths:
        try:
            s = open(p).read()
            print("FOUND:", p)
            print("length:", len(s))
            print(s[:1500])
            found = True
        except Exception as e:
            print("NO:", p, repr(e))
    if not found:
        print("结论：没有在文件系统里找到源码。它可能是 frozen module，编译进固件了。")

def dump_obj(name, obj):
    print("\n---", name, "---")
    print("type:", type(obj))

    if isinstance(obj, (bytes, bytearray)):
        print("repr: <%s len=%d>" % (type(obj).__name__, len(obj)))
        print("first 32 bytes:", [hex(x) for x in obj[:32]])
        return

    print("repr:", obj)

    try:
        print("dir:", dir(obj))
    except Exception as e:
        print("dir ERROR:", repr(e))

    try:
        print("__dict__:", obj.__dict__)
    except Exception as e:
        print("__dict__ ERROR:", repr(e))

def dump_global(name):
    try:
        obj = globals()[name]
        dump_obj(name, obj)
    except Exception as e:
        print("\n---", name, "---")
        print("not found:", repr(e))

section("系统与文件系统")
print("sys.path:", sys.path)
list_dir("/")
list_dir("/app")
list_dir("/cfg")
list_dir("/lib")

section("导入 meowbit")
import meowbit
from meowbit import *

print("meowbit:", meowbit)
print("meowbit.__file__:", getattr(meowbit, "__file__", None))
print("dir(meowbit):", dir(meowbit))

try_read_source("meowbit")

section("meowbit 全局对象")
for name in [
    "i2c", "vspi", "dc", "tft", "fb", "fbuf",
    "screen", "display", "sensor",
    "led1", "led2", "buzzer",
    "sound", "motion", "distance"
]:
    dump_global(name)

section("I2C 扫描")
try:
    print("i2c:", i2c)
    print("scan:", [hex(x) for x in i2c.scan()])
except Exception as e:
    print("I2C scan ERROR:", repr(e))

section("光照/热敏 ADC 对照")
try:
    from machine import ADC, Pin

    adc36 = ADC(Pin(36))
    try:
        adc36.atten(ADC.ATTN_11DB)
    except Exception:
        pass

    adc39 = ADC(Pin(39))
    try:
        adc39.atten(ADC.ATTN_11DB)
    except Exception:
        pass

    try:
        print("sensor.getLight():", sensor.getLight(), "ADC(Pin36).read():", adc36.read())
    except Exception as e:
        print("getLight compare ERROR:", repr(e))

    try:
        print("sensor.getTemp():", sensor.getTemp(), "ADC(Pin39).read():", adc39.read())
    except Exception as e:
        print("getTemp compare ERROR:", repr(e))

except Exception as e:
    print("ADC test ERROR:", repr(e))

section("尝试读取 motor 模块")
try:
    import motor as motor_module
    from motor import Motor

    print("motor module:", motor_module)
    print("motor.__file__:", getattr(motor_module, "__file__", None))
    print("dir(motor):", dir(motor_module))

except Exception as e:
    print("import motor ERROR:", repr(e))

try_read_source("motor")

section("Motor I2C 协议拦截")

class FakeI2C:
    def writeto(self, addr, data):
        print("writeto: addr=%s data=%s" % (hx(addr), hexlist(data)))

    def writeto_mem(self, addr, mem, data):
        print("writeto_mem: addr=%s mem=%s data=%s" % (hx(addr), hx(mem), hexlist(data)))

    def readfrom(self, addr, n):
        print("readfrom: addr=%s n=%d" % (hx(addr), n))
        return bytes([0] * n)

    def readfrom_mem(self, addr, mem, n):
        print("readfrom_mem: addr=%s mem=%s n=%d" % (hx(addr), hx(mem), n))
        return bytes([0] * n)

try:
    m = Motor()
    print("Motor dict before:", m.__dict__)

    # 用 FakeI2C 替换真实 I2C，避免真实驱动电机。
    m.i2c = FakeI2C()
    print("Motor dict after:", m.__dict__)

    print("\nrun(1, 1, 50)")
    m.run(1, 1, 50)

    print("\nrun(1, 0, 50)")
    m.run(1, 0, 50)

    print("\nrun(2, 1, 50)")
    m.run(2, 1, 50)

    print("\nrun(2, 0, 50)")
    m.run(2, 0, 50)

    print("\nstop()")
    m.stop()

    print("\n速度映射测试：motor 1, direction 1")
    for s in [0, 1, 10, 50, 100, 128, 200, 255]:
        print("speed =", s)
        m.run(1, 1, s)

except Exception as e:
    print("Motor probe ERROR:", repr(e))

section("查找 sugar_asr.py 里的 UART/I2C 复用线索")
for p in ["/app/sugar_asr.py", "/cfg/sugar_asr.py", "sugar_asr.py"]:
    try:
        print("checking:", p)
        text = open(p).read()
        for line in text.split("\n"):
            if "UART" in line or "tx=21" in line or "rx=15" in line or "I2C" in line:
                print(line)
    except Exception as e:
        print("NO:", p, repr(e))

section("结束")
print("探查完成。")
screen.fill((0, 170, 170))
screen.textCh(str("成功！"),5,10,2,(170, 0, 0))
screen.refresh()