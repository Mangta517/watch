"""Subset the bundled uncompressed Montserrat fonts without rasterizing them.

Pass an original LVGL src/font directory to restore additional glyphs later.
The 10/14 px fonts, including the Bluetooth receive font, are never modified.
"""
from pathlib import Path
import argparse
import re

ROOT = Path(__file__).resolve().parents[1]
FONT_DIR = ROOT / "Middlewares/LVGL/lvgl/src/font"


def array_body(source, name):
    return re.search(r"\b" + name + r"\[\]\s*=\s*\{(.*?)\};", source, re.S).group(1)


def numbers(body):
    body = re.sub(r"/\*.*?\*/", "", body, flags=re.S)
    return [int(value, 0) for value in re.findall(r"-?0x[0-9a-fA-F]+|-?\d+", body)]


def replace_array(source, name, body):
    pattern = r"(\b" + name + r"\[\]\s*=\s*)\{.*?\};"
    return re.sub(pattern, lambda match: match[1] + "{\n" + body + "\n};", source, count=1, flags=re.S)


def format_values(values):
    return "\n".join("    " + ", ".join(str(value) for value in values[index:index + 16]) + ","
                     for index in range(0, len(values), 16))


def subset_font(source, keep):
    assert re.search(r"\.bitmap_format\s*=\s*0", source), "Requires an uncompressed source font"
    bitmap = numbers(array_body(source, "glyph_bitmap"))
    glyphs = re.findall(r"\{([^{}]+)\}", array_body(source, "glyph_dsc"))
    mapping = {}
    for cmap in re.findall(r"\{([^{}]+)\}", array_body(source, "cmaps")):
        start = int(re.search(r"\.range_start\s*=\s*(\d+)", cmap)[1])
        first = int(re.search(r"\.glyph_id_start\s*=\s*(\d+)", cmap)[1])
        if "SPARSE_TINY" in cmap:
            name = re.search(r"\.unicode_list\s*=\s*(\w+)", cmap)[1]
            points = [start + value for value in numbers(array_body(source, name))]
        else:
            assert "FORMAT0_TINY" in cmap
            length = int(re.search(r"\.range_length\s*=\s*(\d+)", cmap)[1])
            points = list(range(start, start + length))
        mapping.update((point, first + index) for index, point in enumerate(points))

    assert keep <= mapping.keys(), f"Missing glyphs: {sorted(keep - mapping.keys())}"
    points = sorted(keep)
    old_ids = [0] + [mapping[point] for point in points]
    new_bitmap = []
    new_glyphs = ["    {" + glyphs[0].strip() + "},"]
    for point, old_id in zip(points, old_ids[1:]):
        glyph = glyphs[old_id]
        fields = dict((name, int(value)) for name, value in re.findall(r"\.(\w+)\s*=\s*(-?\d+)", glyph))
        start = fields["bitmap_index"]
        size = (fields["box_w"] * fields["box_h"] + 1) // 2
        data = bitmap[start:start + size]
        assert len(data) == size
        glyph = re.sub(r"\.bitmap_index\s*=\s*\d+", f".bitmap_index = {len(new_bitmap)}", glyph)
        new_glyphs.append("    {" + glyph.strip() + f"}}, /* U+{point:04X} */")
        new_bitmap.extend(data)

    # Preserve every remaining pair's original kerning while removing unused classes.
    left = numbers(array_body(source, "kern_left_class_mapping"))
    right = numbers(array_body(source, "kern_right_class_mapping"))
    pairs = numbers(array_body(source, "kern_class_values"))
    old_right_count = int(re.search(r"\.right_class_cnt\s*=\s*(\d+)", source)[1])
    left_classes = sorted({left[index] for index in old_ids} - {0})
    right_classes = sorted({right[index] for index in old_ids} - {0})
    new_left = {value: index + 1 for index, value in enumerate(left_classes)}
    new_right = {value: index + 1 for index, value in enumerate(right_classes)}
    new_pairs = [pairs[(a - 1) * old_right_count + b - 1] for a in left_classes for b in right_classes]
    source = replace_array(source, "kern_left_class_mapping", format_values([new_left.get(left[index], 0) for index in old_ids]))
    source = replace_array(source, "kern_right_class_mapping", format_values([new_right.get(right[index], 0) for index in old_ids]))
    source = replace_array(source, "kern_class_values", format_values(new_pairs))
    source = re.sub(r"\.left_class_cnt\s*=\s*\d+", f".left_class_cnt = {len(left_classes)}", source)
    source = re.sub(r"\.right_class_cnt\s*=\s*\d+", f".right_class_cnt = {len(right_classes)}", source)
    source = replace_array(source, "glyph_bitmap", format_values(new_bitmap))
    source = replace_array(source, "glyph_dsc", "\n".join(new_glyphs))
    source = replace_array(source, "unicode_list_1", format_values([point - points[0] for point in points]))
    source = replace_array(source, "cmaps", "\n".join([
        "    {",
        f"        .range_start = {points[0]}, .range_length = {points[-1] - points[0] + 1}, .glyph_id_start = 1,",
        "        .unicode_list = unicode_list_1, .glyph_id_ofs_list = NULL,",
        f"        .list_length = {len(points)}, .type = LV_FONT_FMT_TXT_CMAP_SPARSE_TINY",
        "    }"]))
    source = re.sub(r"\.cmap_num\s*=\s*\d+", ".cmap_num = 1", source)
    source = re.sub(r"/\* UI subset:.*?\*/\s*", "", source, flags=re.S)
    note = "/* UI subset: tools/subset_ui_fonts.py; original 4-bpp pixels, metrics and kerning retained. */\n"
    return note + source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, default=FONT_DIR)
    args = parser.parse_args()
    ui = (ROOT / "application/app_ui.c").read_text(encoding="utf-8")
    definitions = (FONT_DIR / "lv_symbol_def.h").read_text(encoding="utf-8")
    icons = set()
    for name in set(re.findall(r"\bLV_SYMBOL_\w+", ui)):
        encoded = re.search(r"#define\s+" + name + r'\s+"([^"\n]+)"', definitions)[1]
        icons.add(ord(bytes.fromhex(encoded.replace("\\x", "")).decode("utf-8")))
    # 30 号还显示指南针方向和角度符号；14 号蓝牙文本保留完整字库。
    keep = {20: set(range(32, 127)) | icons, 30: {ord(char) for char in " -.0123456789?NESW\u00b0"}}
    for size, points in keep.items():
        name = f"lv_font_montserrat_{size}.c"
        source = (args.source_dir / name).read_text(encoding="utf-8")
        result = subset_font(source, points)
        with (FONT_DIR / name).open("w", encoding="utf-8", newline="\r\n") as output:
            output.write(result)
        print(f"{name}: retained {len(points)} glyphs at original 4 bpp")


if __name__ == "__main__":
    main()
