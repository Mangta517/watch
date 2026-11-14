/**
 * @file ui_widgets.c
 * @brief 只依赖LVGL的通用控件与趋势图，不持有页面状态。
 */
#include "ui_widgets.h"

#include <math.h>
#include <stdarg.h>
#include <string.h>

/* 直接比较控件中的最终显示值，切页重建后无需维护额外缓存。 */
void ui_set_label_text(lv_obj_t *obj, const char *text)
{
    /* 读取标签当前文本并与目标字符串比较，内容相同就不重复写入。 */
    if (strcmp(lv_label_get_text(obj), text) != 0)
    {
        /* 文本确实变化，才把新字符串交给LVGL标签，触发一次重绘。 */
        lv_label_set_text(obj, text);
    }
}

void ui_set_label_text_fmt(lv_obj_t *obj, const char *format, ...)
{
    /* 当前最长的格式化文本为心率诊断和三轴校准状态，均小于128字节；栈上缓冲在函数返回后失效。 */
    char text[128];
    /* 可变参数表，从format之后的第一个实参开始读取。 */
    va_list args;
    va_start(args, format);
    /* 按format把可变参数写入text，最多sizeof(text)字节，超出部分被截断。 */
    lv_vsnprintf(text, sizeof(text), format, args);
    /* 结束可变参数遍历，释放va_list。 */
    va_end(args);
    /* 把格式化结果交给带内容比较的set_label_text，只在变化时刷新控件。 */
    ui_set_label_text(obj, text);
}

void ui_set_text_color(lv_obj_t *obj, uint32_t color)
{
    /* 把24位颜色值转成LVGL颜色结构，供比较和目标色共用。 */
    lv_color_t desired = lv_color_hex(color);
    /* 读取MAIN部件当前文字色的full值，与目标色相同则跳过设置。 */
    if (lv_obj_get_style_text_color(obj, LV_PART_MAIN).full != desired.full)
    {
        /* 颜色不同，更新MAIN部件的文字颜色。 */
        lv_obj_set_style_text_color(obj, desired, LV_PART_MAIN);
    }
}

/* Explicit styles keep the built-in light theme out of the entire UI. */
lv_obj_t *ui_panel(lv_obj_t *parent, int x, int y, int w, int h, uint32_t color)
{
    /* 在parent下创建基础对象，后续作为卡片或容器使用。 */
    lv_obj_t *obj = lv_obj_create(parent);
    /* 清掉LVGL默认主题样式，避免浅色主题的背景和边框混入。 */
    lv_obj_remove_style_all(obj);
    /* 设置相对父对象的左上角坐标(x,y)，单位像素。 */
    lv_obj_set_pos(obj, x, y);
    /* 设置对象宽w、高h，单位像素。 */
    lv_obj_set_size(obj, w, h);
    /* 主部件背景填充为传入的24位颜色。 */
    lv_obj_set_style_bg_color(obj, lv_color_hex(color), 0);
    /* 背景不透明度设为完全覆盖，遮住下层内容。 */
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    /* 圆角半径16像素，形成卡片外观。 */
    lv_obj_set_style_radius(obj, 16, 0);
    /* 去掉可滚动标志，容器内容不跟随手指滚动。 */
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    /* 返回创建好的容器，调用方继续添加子控件。 */
    return obj;
}

lv_obj_t *ui_label(lv_obj_t *parent, const char *text, int x, int y, const lv_font_t *font, uint32_t color)
{
    /* 在parent下创建标签对象。 */
    lv_obj_t *obj = lv_label_create(parent);
    /* 清除标签默认样式，只保留下面显式设置的字体、颜色和位置。 */
    lv_obj_remove_style_all(obj);
    /* 指定文本字体，例如montserrat_10/14/20/30。 */
    lv_obj_set_style_text_font(obj, font, 0);
    /* 指定文本颜色。 */
    lv_obj_set_style_text_color(obj, lv_color_hex(color), 0);
    /* 设置标签相对父对象的坐标。 */
    lv_obj_set_pos(obj, x, y);
    /* 写入要显示的字符串。 */
    lv_label_set_text(obj, text);
    /* 返回标签对象，便于调用方继续调整宽度或对齐。 */
    return obj;
}

lv_obj_t *ui_note(lv_obj_t *parent, const char *text, int y)
{
    /* 用10号灰色字体创建说明标签，x固定为4，y由调用方给出。 */
    lv_obj_t *obj = ui_label(parent, text, 4, y, &lv_font_montserrat_10, UI_MUTED);
    /* 限制宽度204像素，与页面内容宽度对应。 */
    lv_obj_set_width(obj, 204);
    /* 超宽文本改为自动换行，避免横向溢出。 */
    lv_label_set_long_mode(obj, LV_LABEL_LONG_WRAP);
    /* 返回标签供调用方继续使用。 */
    return obj;
}

/* 心率每秒、海拔每10分钟记录，最右为最新值；海拔历史由sensor持续维护。 */
void ui_trend_create(ui_trend_struct *trend, lv_obj_t *parent, const char *title, int y, int height, uint32_t color, uint32_t sample_ms)
{
    /* 在parent的y位置建高度为height的卡片，承载标题、刻度和图表。 */
    lv_obj_t *card = ui_panel(parent, 0, y, 212, height, UI_CARD);
    /* 图表下方采样间隔说明标签。 */
    lv_obj_t *caption;
    /* 卡片左上角写曲线标题，使用该曲线的主题色。 */
    ui_label(card, title, 12, 8, &lv_font_montserrat_10, color);
    /* 纵轴上端刻度标签，初值"--"。 */
    trend->scale = ui_label(card, "--", 0, 23, &lv_font_montserrat_10, UI_MUTED);
    /* 纵轴下端刻度标签，放在卡片底部上方31像素处。 */
    trend->scale_low = ui_label(card, "--", 0, height - 31, &lv_font_montserrat_10, UI_MUTED);
    /* 两个刻度标签宽度固定36像素。 */
    lv_obj_set_width(trend->scale, 36);
    lv_obj_set_width(trend->scale_low, 36);
    /* 刻度文字右对齐，贴近图表左边界。 */
    lv_obj_set_style_text_align(trend->scale, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_align(trend->scale_low, LV_TEXT_ALIGN_RIGHT, 0);
    /* 底部左侧建采样间隔说明标签，先置空文本。 */
    caption = ui_label(card, "", 42, height - 17, &lv_font_montserrat_10, UI_MUTED);
    /* 采样周期达到1分钟时按分钟显示，每格对应sample_ms/60000分钟。 */
    if (sample_ms >= 60000)
    {
        ui_set_label_text_fmt(caption, "%lumin / bar", (unsigned long)(sample_ms / 60000));
    }
    else
    {
        /* 不足1分钟时按秒显示，每格对应sample_ms/1000秒。 */
        ui_set_label_text_fmt(caption, "%lus / bar", (unsigned long)(sample_ms / 1000));
    }
    /* 底部右侧标注最新值方向。 */
    ui_label(card, "NOW", 172, height - 17, &lv_font_montserrat_10, UI_MUTED);
    /* 创建图表对象并清掉默认样式，尺寸与背景完全由下面设置决定。 */
    trend->chart = lv_chart_create(card);
    lv_obj_remove_style_all(trend->chart);
    /* 图表定位到卡片内(42,28)，尺寸156 x (height-50)，左侧留给刻度文字。 */
    lv_obj_set_pos(trend->chart, 42, 28);
    lv_obj_set_size(trend->chart, 156, height - 50);
    /* 图表不滚动也不接收点击，避免抢页面手势。 */
    lv_obj_clear_flag(trend->chart, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    /* 主区域边框线用UI_BORDER色、1像素宽。 */
    lv_obj_set_style_line_color(trend->chart, lv_color_hex(UI_BORDER), LV_PART_MAIN);
    lv_obj_set_style_line_width(trend->chart, 1, LV_PART_MAIN);
    /* 柱形填充不透明，柱间距2像素。 */
    lv_obj_set_style_bg_opa(trend->chart, LV_OPA_COVER, LV_PART_ITEMS);
    lv_obj_set_style_pad_column(trend->chart, 2, LV_PART_ITEMS);
    /* 图表类型设为柱状。 */
    lv_chart_set_type(trend->chart, LV_CHART_TYPE_BAR);
    /* 固定保存UI_TREND_POINTS个点，滚动覆盖最旧值。 */
    lv_chart_set_point_count(trend->chart, UI_TREND_POINTS);
    /* 只画3条水平分割线，不画垂直分割线。 */
    lv_chart_set_div_line_count(trend->chart, 3, 0);
    /* 主Y轴初始范围0..200，随后由trend_update_range按数据调整。 */
    lv_chart_set_range(trend->chart, LV_CHART_AXIS_PRIMARY_Y, 0, 200);
    /* 给图表添加使用主题色的主Y轴数据序列。 */
    trend->series = lv_chart_add_series(trend->chart, lv_color_hex(color), LV_CHART_AXIS_PRIMARY_Y);
    /* 记录采样周期，trend_update据此限速。 */
    trend->sample_period_ms = sample_ms;
    /* 把上次采样时刻回拨一个周期，使第一次更新立即写入一个点。 */
    trend->sample_tick = lv_tick_get() - sample_ms;
}

void ui_trend_update_range(ui_trend_struct *trend, uint8_t altitude)
{
    /* 纵轴初始按0..200，心率等非海拔曲线直接使用。 */
    lv_coord_t low = 0, high = 200;
    /* 序列点下标，遍历全部UI_TREND_POINTS个点。 */
    uint16_t i;
    /* altitude非0时按海拔窗口动态计算上下界；为0则保持上面的固定范围。 */
    if (altitude)
    {
        /* low先置"无点"哨兵，high先置很小的初值，两者随后被窗口内各点刷新。 */
        low = LV_CHART_POINT_NONE;
        high = -30000;
        /* 逐点读取序列缓存，跳过尚未写入的点。 */
        for (i = 0; i < UI_TREND_POINTS; ++i)
        {
            /* 取第i个柱形的纵坐标。 */
            lv_coord_t point = trend->series->y_points[i];
            /* 有实际值的点才参与最小/最大统计。 */
            if (point != LV_CHART_POINT_NONE)
            {
                /* 更新窗口下界。 */
                low = LV_MIN(low, point);
                /* 更新窗口上界。 */
                high = LV_MAX(high, point);
            }
        }
        /* 整窗都没有数据时把上下刻度都显示为"--"，保持原图表范围并返回。 */
        if (low == LV_CHART_POINT_NONE)
        {
            ui_set_label_text(trend->scale, "--");
            ui_set_label_text(trend->scale_low, "--");
            return;
        }
        /* 海拔允许负值，纵轴跟随窗口并保留至少20m跨度以免放大静止噪声。 */
        low -= 10;
        high += 10;
    }
    /* 上端刻度显示high，下端显示low，均为整数米/单位。 */
    ui_set_label_text_fmt(trend->scale, "%d", (int)high);
    ui_set_label_text_fmt(trend->scale_low, "%d", (int)low);
    /* 把算出的上下界写回图表主Y轴。 */
    lv_chart_set_range(trend->chart, LV_CHART_AXIS_PRIMARY_Y, low, high);
}

void ui_trend_update(ui_trend_struct *trend, lv_coord_t value, uint8_t altitude)
{
    /* 取当前LVGL毫秒tick作为本轮采样时刻。 */
    uint32_t now = lv_tick_get();
    /* 图表尚未创建，或距上次采样不足一个周期时跳过，不推进图表。 */
    if (trend->chart == NULL || (uint32_t)(now - trend->sample_tick) < trend->sample_period_ms)
    {
        return;
    }
    /* 记录本轮采样时刻，作为下一次限速基准。 */
    trend->sample_tick = now;
    /* 把新值压入序列，最旧点被覆盖，最右为最新。 */
    lv_chart_set_next_value(trend->chart, trend->series, value);
    /* 按新数据重算纵轴范围，心率用固定0..200，海拔按窗口。 */
    ui_trend_update_range(trend, altitude);
}

void ui_set_status(lv_obj_t *obj, uint8_t online)
{
    /* 控件指针为空说明当前页没有这个标签，直接返回。 */
    if (obj == NULL)
    {
        return;
    }
    /* online非0时显示ONLINE标签，0时显示NO DATA标签。 */
    if (online)
    {
        ui_set_label_text(obj, "ONLINE");
        ui_set_text_color(obj, UI_GREEN);
    }
    /* online为0时显示NO DATA标签，用红色提示。 */
    else
    {
        ui_set_label_text(obj, "NO DATA");
        ui_set_text_color(obj, UI_RED);
    }
}

/* Fixed-point text avoids pulling printf floating-point support into firmware.
 * 把浮点数格式化为一位小数文本：先放大10倍转整数再拆整数/小数部分，避免引入printf浮点支持。 */
void ui_set_decimal(lv_obj_t *obj, float value, uint8_t wrap)
{
    /* 放大10倍后的有符号定点值，用于拆出整数位和小数位。 */
    int32_t fixed;
    /* fixed的绝对值，保证拆分时不带符号。 */
    int32_t magnitude;
    /* 输出用符号串，负数时为"-"，否则为空串。 */
    const char *sign;
    /* 非有限值直接显示"--"；wrap为0且绝对值超过99999时同样视为不可显示。 */
    if (!isfinite(value) || (!wrap && (value > 99999.0f || value < -99999.0f)))
    {
        ui_set_label_text(obj, "--");
        return;
    }
    /* wrap非0表示按角度处理，先把值折到一圈内。 */
    if (wrap)
    {
        /* 对360取模，结果符号跟随被除数。 */
        value = fmodf(value, 360.0f);
        /* 大于180度的部分减360，折到(-180,180]。 */
        if (value > 180.0f)
        {
            value -= 360.0f;
        }
        /* 小于等于-180度的部分加360，避免出现-180整点重复。 */
        if (value <= -180.0f)
        {
            value += 360.0f;
        }
    }
    /* 乘10后强制转int32，向零截断，只保留一位小数。 */
    fixed = (int32_t)(value * 10.0f);
    /* 负数取绝对值，交给magnitude做无符号拆分。 */
    if (fixed < 0)
    {
        magnitude = -fixed;
    }
    /* 非负数直接作为幅值。 */
    else
    {
        magnitude = fixed;
    }
    /* 符号按wrap处理后的value判断，-0.x这类定点截断成0的值仍会带负号。 */
    if (value < 0.0f)
    {
        sign = "-";
    }
    /* 非负值不加符号。 */
    else
    {
        sign = "";
    }
    /* 整数部分为magnitude/10，小数部分为magnitude%10，按"%s%ld.%ld"拼成一位小数。 */
    ui_set_label_text_fmt(obj, "%s%ld.%ld", sign, (long)(magnitude / 10), (long)(magnitude % 10));
}
