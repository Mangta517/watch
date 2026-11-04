"""检查build.py的BleMode DUAL路由补丁：只改transport_type、可重复执行、映射为0/1/2/2。"""
import os
import shutil
import sys
import tempfile
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import build


OLD_CLASS = """.class public final Ldev/mangta/wave/BleMode;
.super Ljava/lang/Object;
.source "BleMode.java"


# direct methods
.method private constructor <init>()V
    .locals 0

    .line 7
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 8
    return-void
.end method

.method public static transport_type(I)I
    .locals 2

    .line 12
    const/4 v0, 0x2

    if-ne p0, v0, :cond_0

    .line 14
    return v0

    .line 16
    :cond_0
    const/4 v0, 0x1

    if-eq p0, v0, :cond_2

    const/4 v1, 0x3

    if-ne p0, v1, :cond_1

    goto :goto_0

    .line 20
    :cond_1
    const/4 p0, 0x0

    return p0

    .line 18
    :cond_2
    :goto_0
    return v0
.end method
"""


def method_text(text):
    """取出transport_type方法原文，用于比较补丁范围。"""
    start = text.index(".method public static transport_type(I)I")
    end = text.index(".end method", start) + len(".end method")
    return text[start:end]


def registers(instruction):
    """把smali指令拆成操作码和操作数；标签前的冒号单独去掉。"""
    parts = instruction.replace(",", " ").split()
    return parts[0], parts[1:]


def label(operand):
    """条件跳转和goto的操作数带冒号，标签表统一按名字索引。"""
    if operand.startswith(":"):
        return operand[1:]
    return operand


def run_transport_type(text, argument):
    """只解释该方法用到的const/4、if-ne、if-eq、goto和return，求实际返回值。"""
    body = method_text(text)
    labels = {}
    instructions = []
    for raw in body.splitlines():
        line = raw.strip()
        if line.startswith(":") and " " not in line:
            labels[line[1:]] = len(instructions)
            continue
        if line.startswith(".") or line == "":
            continue
        instructions.append(line)
    values = {"p0": argument, "v0": 0, "v1": 0}
    pc = 0
    for _ in range(256):
        if pc < 0 or pc >= len(instructions):
            raise AssertionError("transport_type fell off the method body")
        opcode, operands = registers(instructions[pc])
        if opcode == "const/4":
            values[operands[0]] = int(operands[1], 0)
            pc += 1
        elif opcode == "return":
            return values[operands[0]]
        elif opcode == "goto":
            pc = labels[label(operands[0])]
        elif opcode == "if-ne" or opcode == "if-eq":
            left = values[operands[0]]
            right = values[operands[1]]
            taken = left != right
            if opcode == "if-eq":
                taken = left == right
            if taken:
                pc = labels[label(operands[2])]
            else:
                pc += 1
        else:
            raise AssertionError("unexpected opcode " + opcode)
    raise AssertionError("transport_type did not return")


def decode_mapping(text):
    """按真实方法文本求0/1/2/3的路由结果。"""
    result = []
    for argument in (0, 1, 2, 3):
        result.append(run_transport_type(text, argument))
    return result


def write_fixture(root, text):
    path = root / "smali_classes2/dev/mangta/wave/BleMode.smali"
    path.parent.mkdir(parents=True)
    path.write_bytes(text.encode("utf-8"))
    return path


def check_old_fixture(root):
    """确认测试夹具就是升级前的真实路由：DUAL(3)仍返回经典蓝牙1。"""
    path = write_fixture(root, OLD_CLASS)
    original = path.read_bytes().decode("utf-8")
    assert original == OLD_CLASS, "fixture changed while writing"
    assert decode_mapping(original) == [0, 1, 2, 1], "old fixture is not the known DUAL routing"
    assert "0x3" in method_text(original), "old fixture must contain the DUAL branch"


def check_patch(root):
    """补丁后只改transport_type，且0->0、1->1、2->2、3->2。"""
    path = write_fixture(root, OLD_CLASS)
    assert build.patch_ble_mode(root) is True, "first patch must report a change"
    patched = path.read_bytes().decode("utf-8")
    assert patched != OLD_CLASS, "patch did not change the file"
    assert patched.count(build.TRANSPORT_METHOD_NEW) == 1, "new method must appear once"
    assert build.TRANSPORT_METHOD_OLD not in patched, "old method must be gone"
    assert method_text(patched) == build.TRANSPORT_METHOD_NEW, "only transport_type may change"
    assert patched.replace(build.TRANSPORT_METHOD_NEW, build.TRANSPORT_METHOD_OLD) == OLD_CLASS, \
        "patch touched text outside transport_type"
    assert decode_mapping(patched) == [0, 1, 2, 2], "DUAL must route back to BLE"
    assert decode_mapping(patched)[1] == 1, "classic-only support must stay 1"
    return patched


def check_idempotent(root):
    """第二次执行不得再改文件，也不得报错。"""
    path = write_fixture(root, OLD_CLASS)
    assert build.patch_ble_mode(root) is True, "first patch must report a change"
    once = path.read_bytes()
    assert build.patch_ble_mode(root) is False, "second patch must be a no-op"
    assert path.read_bytes() == once, "second patch rewrote the file"


def check_crlf(root):
    """真实解码文件是CRLF，补丁必须保留原换行约定。"""
    crlf = OLD_CLASS.replace("\n", "\r\n")
    path = write_fixture(root, crlf)
    assert build.patch_ble_mode(root) is True, "CRLF fixture must be patched"
    raw = path.read_bytes().decode("utf-8")
    assert "\r\n" in raw and "\n" not in raw.replace("\r\n", ""), "CRLF was not preserved"
    patched = raw.replace("\r\n", "\n")
    assert patched.replace(build.TRANSPORT_METHOD_NEW, build.TRANSPORT_METHOD_OLD) == OLD_CLASS, \
        "CRLF patch touched text outside transport_type"
    assert decode_mapping(patched) == [0, 1, 2, 2], "CRLF DUAL mapping is wrong"
    assert build.patch_ble_mode(root) is False, "CRLF second patch must be a no-op"
    assert path.read_bytes().decode("utf-8") == raw, "CRLF second patch rewrote the file"


def check_rejects_unknown(root):
    """方法缺失或不是唯一一份时必须拒绝，不能盲改其他代码。"""
    missing = root / "missing"
    missing.mkdir()
    try:
        build.patch_ble_mode(missing)
    except RuntimeError:
        pass
    else:
        raise AssertionError("missing BleMode.smali must be rejected")
    duplicate = write_fixture(root, OLD_CLASS + "\n" + OLD_CLASS)
    try:
        build.patch_ble_mode(root)
    except RuntimeError:
        pass
    else:
        raise AssertionError("duplicated transport_type must be rejected")
    assert duplicate.read_bytes().decode("utf-8") == OLD_CLASS + "\n" + OLD_CLASS, \
        "rejected patch must not modify the file"


def check_actual_fixture():
    """若本机存在真实解码文件，只读核对夹具方法文本，确认不是凭空编造。"""
    repo = Path(__file__).resolve().parents[3]
    paths = sorted(repo.glob("work/*/android-build/decoded/smali_classes2/dev/mangta/wave/BleMode.smali"))
    for path in paths:
        text = path.read_bytes().decode("utf-8").replace("\r\n", "\n")
        current = method_text(text)
        assert current == build.TRANSPORT_METHOD_OLD or current == build.TRANSPORT_METHOD_NEW, \
            "decoded BleMode transport_type differs from both known versions: " + str(path)
        if current == build.TRANSPORT_METHOD_OLD:
            assert current == method_text(OLD_CLASS), "embedded old fixture drifted from " + str(path)
    return len(paths)


def main():
    """用系统临时目录下的副本运行各场景，结束后删除，不触碰真实decoded输出。"""
    scratch = Path(tempfile.gettempdir()) / ("build-routing-" + uuid.uuid4().hex[:8])
    os.mkdir(scratch)
    try:
        for name in ("old", "patch", "idempotent", "crlf", "reject"):
            (scratch / name).mkdir()
        check_old_fixture(scratch / "old")
        check_patch(scratch / "patch")
        check_idempotent(scratch / "idempotent")
        check_crlf(scratch / "crlf")
        check_rejects_unknown(scratch / "reject")
    finally:
        shutil.rmtree(scratch, ignore_errors=True)
    actual = check_actual_fixture()
    print("PASS: BleMode DUAL patch changes only transport_type once, is idempotent, keeps CRLF, "
          "maps 0/1/2/3 to 0/1/2/2, and rejects missing or duplicated methods; "
          "checked " + str(actual) + " decoded fixture(s)")


if __name__ == "__main__":
    main()
