#!/usr/bin/env python3
"""Synthesize an H.264 SPS+PPS for a known geometry and prepend them to a raw IDR slice.

The Dreame CedarX encoder writes valid IDR/P slices but (in this build) never materializes
the SPS/PPS header bytes, so a decoder can't render the stream. The slice was encoded against
the encoder's *internal* SPS values; we reconstruct a matching SPS/PPS. Geometry is known
(672x504 -> 42x32 macroblocks, 8px bottom crop). The only fields that affect slice parsing are
log2_max_frame_num and pic_order_cnt_type, which we sweep and verify by decoding.

Usage: h264_headers.py <idr.h264> <out.h264> W H profile log2_maxfn_m4 poc_type poc_lsb_m4
"""
import sys

class BW:
    def __init__(self): self.bits = []
    def u(self, n, v):
        for i in range(n - 1, -1, -1): self.bits.append((v >> i) & 1)
    def ue(self, v):
        v1 = v + 1; n = v1.bit_length()
        self.u(n - 1, 0); self.u(n, v1)
    def se(self, v):
        self.ue(0 if v == 0 else (2 * v - 1 if v > 0 else -2 * v))
    def rbsp(self):
        self.bits.append(1)
        while len(self.bits) % 8: self.bits.append(0)
        out = bytearray()
        for i in range(0, len(self.bits), 8):
            b = 0
            for k in range(8): b = (b << 1) | self.bits[i + k]
            out.append(b)
        return bytes(out)

def emulation_prevent(rbsp):
    out = bytearray(); zeros = 0
    for b in rbsp:
        if zeros >= 2 and b <= 3:
            out.append(3); zeros = 0
        out.append(b)
        zeros = zeros + 1 if b == 0 else 0
    return bytes(out)

def sps(W, H, profile, log2_maxfn_m4, poc_type, poc_lsb_m4, level=30):
    mbw = (W + 15) // 16; mbh = (H + 15) // 16
    crop_b = (mbh * 16 - H) // 2
    b = BW()
    b.u(8, profile)            # profile_idc
    b.u(8, 0xE0 if profile == 66 else 0)  # constraint flags
    b.u(8, level)              # level_idc
    b.ue(0)                    # seq_parameter_set_id
    b.ue(log2_maxfn_m4)        # log2_max_frame_num_minus4
    b.ue(poc_type)             # pic_order_cnt_type
    if poc_type == 0:
        b.ue(poc_lsb_m4)       # log2_max_pic_order_cnt_lsb_minus4
    b.ue(1)                    # max_num_ref_frames
    b.u(1, 0)                  # gaps_in_frame_num_value_allowed
    b.ue(mbw - 1)              # pic_width_in_mbs_minus1
    b.ue(mbh - 1)              # pic_height_in_map_units_minus1
    b.u(1, 1)                  # frame_mbs_only_flag
    b.u(1, 0)                  # direct_8x8_inference_flag
    crop = 1 if crop_b else 0
    b.u(1, crop)               # frame_cropping_flag
    if crop:
        b.ue(0); b.ue(0); b.ue(0); b.ue(crop_b)
    b.u(1, 0)                  # vui_parameters_present_flag
    return b"\x00\x00\x00\x01\x67" + emulation_prevent(b.rbsp())

def pps():
    b = BW()
    b.ue(0)        # pic_parameter_set_id
    b.ue(0)        # seq_parameter_set_id
    b.u(1, 0)      # entropy_coding_mode_flag (CAVLC)
    b.u(1, 0)      # bottom_field_pic_order_in_frame_present
    b.ue(0)        # num_slice_groups_minus1
    b.ue(0)        # num_ref_idx_l0_default_active_minus1
    b.ue(0)        # num_ref_idx_l1_default_active_minus1
    b.u(1, 0)      # weighted_pred_flag
    b.u(2, 0)      # weighted_bipred_idc
    b.se(0)        # pic_init_qp_minus26
    b.se(0)        # pic_init_qs_minus26
    b.se(0)        # chroma_qp_index_offset
    b.u(1, 0)      # deblocking_filter_control_present
    b.u(1, 0)      # constrained_intra_pred_flag
    b.u(1, 0)      # redundant_pic_cnt_present
    return b"\x00\x00\x00\x01\x68" + emulation_prevent(b.rbsp())

def main():
    idr, out, W, H, prof, l2, poc, plsb = (sys.argv[1], sys.argv[2],
        int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]),
        int(sys.argv[6]), int(sys.argv[7]), int(sys.argv[8]))
    slice_ = open(idr, "rb").read()
    with open(out, "wb") as f:
        f.write(sps(W, H, prof, l2, poc, plsb)); f.write(pps()); f.write(slice_)

if __name__ == "__main__":
    main()
