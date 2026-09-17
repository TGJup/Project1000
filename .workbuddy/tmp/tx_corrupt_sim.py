# -*- coding: utf-8 -*-
"""
仿真：HAL_UART_Transmit_IT 是"指针 + 边发边读"，不是拷贝。
验证原版 FC_Packet_Make（先改缓冲区、后判忙）会怎样污染空中帧。
"""
BAUD = 115200
BYTE_US = 10.0 * 1e6 / BAUD          # 8N1 = 10 bit/字节  -> 86.8us
BYTE_MS = BYTE_US / 1000.0
FRAME = 19                            # FC_Packet_Len
TX_MS = FRAME * BYTE_MS               # 一帧完全移出的时间 = 1.649ms

FC_SEND_CYCLE = 80                    # ms
FC_ZERO_TIME = 3000                   # ms


def make_frame(vals):
    """EB FE + 8x16bit(大端) + 校验和(byte[0..17]求和%256)"""
    f = [0xEB, 0xFE]
    for v in vals:
        f += [(v >> 8) & 0xFF, v & 0xFF]
    f.append(sum(f) % 256)
    return f


ZERO_FRAME = make_frame([0] * 8)
# 一帧正常数据：Vbus12=280(28.0V) / Vchn8=3000 / Ichn8=12345 / Ichn7=0 / Ichn6=500 / Ichn5=600
NORM_FRAME = make_frame([280, 3000, 12345, 0, 500, 600, 15000, 33000])

print('零帧  :', ' '.join('%02X' % b for b in ZERO_FRAME))
print('正常帧:', ' '.join('%02X' % b for b in NORM_FRAME))
print('一帧发送耗时 = %.3f ms (= %d 字节 x %.1f us)  <- 这就是"缓冲区不能动"的窗口宽度'
      % (TX_MS, FRAME, BYTE_US))
print()


def sim(loop_ms, duration_ms, send_frame, trace_from=None):
    """返回 (发送成功次数, 忙时改写次数, 空中字节流)"""
    buf = [0] * FRAME          # USART1_TXBuffer
    st = {'busy': False, 'idx': 0, 'left': 0}
    wire = []
    t = 0.0
    next_loop = 0.0
    next_byte = None
    ok = 0
    hit_busy = 0

    def txe():
        wire.append(buf[st['idx']])       # 就是这一句：发送时从你的缓冲区读
        st['idx'] += 1
        st['left'] -= 1
        if st['left'] == 0:
            st['busy'] = False

    while t < duration_ms:
        nxt = next_loop
        if next_byte is not None and next_byte < nxt:
            nxt = next_byte
        t = nxt
        if t >= duration_ms:
            break
        if next_byte is not None and abs(t - next_byte) < 1e-9:
            txe()
            next_byte = t + BYTE_MS if st['left'] > 0 else None
            continue
        # ---- 主循环一轮：等价原版 FC_Packet_Make ----
        for i, b in enumerate(send_frame):
            buf[i] = b                     # ① 无条件改写缓冲区
        if st['busy']:                     # ② 这里才判忙
            hit_busy += 1
            if trace_from is not None and not wire:
                trace_from['buf_at_reject'] = list(buf)
        else:
            st['busy'] = True
            st['idx'] = 0
            st['left'] = FRAME
            next_byte = t
            ok += 1
        next_loop = t + loop_ms
    return ok, hit_busy, wire


print('=' * 74)
print('【实验一】零帧阶段（原版逻辑：缺 U1_last_tick 刷新 -> tick_diff 恒成立，每轮主循环都尝试发）')
print('=' * 74)
print('%12s %14s %16s' % ('主循环周期', '实际送出帧数', '忙时改写缓冲区次数'))
for lp in (0.1, 0.5, 1.0, 1.6, 2.0, 5.0):
    ok, hb, _ = sim(lp, FC_ZERO_TIME, ZERO_FRAME)
    flag = '   <-- 缓冲区被改在空中！' if hb else ''
    print('%10.1fms %14d %16d%s' % (lp, ok, hb, flag))
print()
print('判据：只要"主循环周期 < 一帧发送耗时(%.3fms)"，每轮都会撞上正在发送的缓冲区。' % TX_MS)
print()

print('=' * 74)
print('【实验二】把"撞上"那一刻的空中字节流抓出来看')
print('=' * 74)
# 构造：先让零帧开始发送，发到第 5 个字节时主循环拿正常帧来打包
buf = [0] * FRAME
st = {'busy': False, 'idx': 0, 'left': 0}
wire = []


def push(frame, t):
    global wire
    for i, b in enumerate(frame):
        buf[i] = b
    if st['busy']:
        return False
    st['busy'], st['idx'], st['left'] = True, 0, FRAME
    return True


push(ZERO_FRAME, 0.0)                       # 零帧开始发送
for _ in range(5):                          # 移出 5 个字节
    wire.append(buf[st['idx']]); st['idx'] += 1; st['left'] -= 1
push(NORM_FRAME, 5 * BYTE_MS)               # 第 6 个字节前，主循环改写缓冲区
while st['left'] > 0:                       # 剩下的字节继续从(已被改写的)缓冲区取
    wire.append(buf[st['idx']]); st['idx'] += 1; st['left'] -= 1

print('第 1~5 字节 取自【零帧】，第 6~19 字节 取自【正常帧】：')
print('  空中实收: ', ' '.join('%02X' % b for b in wire))
print('  零帧    : ', ' '.join('%02X' % b for b in ZERO_FRAME))
print('  正常帧  : ', ' '.join('%02X' % b for b in NORM_FRAME))
print()
head_ok = wire[0] == 0xEB and wire[1] == 0xFE
ck = sum(wire[0:18]) % 256
print('  帧头 EB FE        : %s' % ('通过' if head_ok else '失败'))
print('  校验和 byte[18]   : 收到 0x%02X，正确应为 0x%02X  ->  %s'
      % (wire[18], ck, '校验通过' if wire[18] == ck else '★ 校验失败，飞控会丢弃这一帧'))

print()
print('=' * 74)
print('【实验三】正常通讯阶段、80ms 节拍稳态（跑 8 秒）')
print('=' * 74)
print('%12s %12s %10s %16s' % ('主循环周期', '送出帧数', '理论值', '忙时改写次数'))
for lp in (0.1, 0.5, 1.0, 1.6, 2.0, 5.0):
    buf2 = [0] * FRAME
    s2 = {'busy': False, 'idx': 0, 'left': 0}
    ok = hb = 0
    t = 0.0
    next_loop = 0.0
    next_byte = None
    last = 0.0
    DUR = 8000.0

    def txe2():
        s2['idx'] += 1
        s2['left'] -= 1
        if s2['left'] == 0:
            s2['busy'] = False

    while t < DUR:
        nxt = next_loop
        if next_byte is not None and next_byte < nxt:
            nxt = next_byte
        t = nxt
        if t >= DUR:
            break
        if next_byte is not None and abs(t - next_byte) < 1e-9:
            txe2()
            next_byte = t + BYTE_MS if s2['left'] > 0 else None
            continue
        if t - last >= FC_SEND_CYCLE:          # 节拍到点
            for i, b in enumerate(NORM_FRAME):
                buf2[i] = b                    # 原版：先无条件改写
            if s2['busy']:
                hb += 1
            else:
                s2['busy'], s2['idx'], s2['left'] = True, 0, FRAME
                next_byte = t
                ok += 1
            last = t                           # 原版：无论发没发出去都推进时间戳
        next_loop = t + lp
    print('%10.1fms %12d %10d %16d%s'
          % (lp, ok, int(DUR / FC_SEND_CYCLE), hb, '  <-- 撞上！' if hb else ''))
print()
print('说明：正常阶段"距上次发送 >= 80ms"这个门限，本身就把 1.649ms 的危险窗口躲开了，')
print('      所以固定节拍下确实撞不上 —— 你在这个点上的直觉是对的。')
