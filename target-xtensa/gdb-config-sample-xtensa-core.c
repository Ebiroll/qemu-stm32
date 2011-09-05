/* Configuration for the Xtensa architecture for GDB, the GNU debugger.

   Copyright (c) 2003-2010 Tensilica Inc.

   Permission is hereby granted, free of charge, to any person obtaining
   a copy of this software and associated documentation files (the
   "Software"), to deal in the Software without restriction, including
   without limitation the rights to use, copy, modify, merge, publish,
   distribute, sublicense, and/or sell copies of the Software, and to
   permit persons to whom the Software is furnished to do so, subject to
   the following conditions:

   The above copyright notice and this permission notice shall be included
   in all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
   EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
   MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
   CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
   TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.  */

  XTREG(0,   0, 32, 4, 4, 0x0020, 0x0006, -2, 9, 0x0100, pc,
          0, 0, 0, 0, 0, 0)
  XTREG(1,   4, 32, 4, 4, 0x0100, 0x0006, -2, 1, 0x0002, ar0,
          0, 0, 0, 0, 0, 0)
  XTREG(2,   8, 32, 4, 4, 0x0101, 0x0006, -2, 1, 0x0002, ar1,
          0, 0, 0, 0, 0, 0)
  XTREG(3,  12, 32, 4, 4, 0x0102, 0x0006, -2, 1, 0x0002, ar2,
          0, 0, 0, 0, 0, 0)
  XTREG(4,  16, 32, 4, 4, 0x0103, 0x0006, -2, 1, 0x0002, ar3,
          0, 0, 0, 0, 0, 0)
  XTREG(5,  20, 32, 4, 4, 0x0104, 0x0006, -2, 1, 0x0002, ar4,
          0, 0, 0, 0, 0, 0)
  XTREG(6,  24, 32, 4, 4, 0x0105, 0x0006, -2, 1, 0x0002, ar5,
          0, 0, 0, 0, 0, 0)
  XTREG(7,  28, 32, 4, 4, 0x0106, 0x0006, -2, 1, 0x0002, ar6,
          0, 0, 0, 0, 0, 0)
  XTREG(8,  32, 32, 4, 4, 0x0107, 0x0006, -2, 1, 0x0002, ar7,
          0, 0, 0, 0, 0, 0)
  XTREG(9,  36, 32, 4, 4, 0x0108, 0x0006, -2, 1, 0x0002, ar8,
          0, 0, 0, 0, 0, 0)
  XTREG(10,  40, 32, 4, 4, 0x0109, 0x0006, -2, 1, 0x0002, ar9,
          0, 0, 0, 0, 0, 0)
  XTREG(11,  44, 32, 4, 4, 0x010a, 0x0006, -2, 1, 0x0002, ar10,
          0, 0, 0, 0, 0, 0)
  XTREG(12,  48, 32, 4, 4, 0x010b, 0x0006, -2, 1, 0x0002, ar11,
          0, 0, 0, 0, 0, 0)
  XTREG(13,  52, 32, 4, 4, 0x010c, 0x0006, -2, 1, 0x0002, ar12,
          0, 0, 0, 0, 0, 0)
  XTREG(14,  56, 32, 4, 4, 0x010d, 0x0006, -2, 1, 0x0002, ar13,
          0, 0, 0, 0, 0, 0)
  XTREG(15,  60, 32, 4, 4, 0x010e, 0x0006, -2, 1, 0x0002, ar14,
          0, 0, 0, 0, 0, 0)
  XTREG(16,  64, 32, 4, 4, 0x010f, 0x0006, -2, 1, 0x0002, ar15,
          0, 0, 0, 0, 0, 0)
  XTREG(17,  68, 32, 4, 4, 0x0110, 0x0006, -2, 1, 0x0002, ar16,
          0, 0, 0, 0, 0, 0)
  XTREG(18,  72, 32, 4, 4, 0x0111, 0x0006, -2, 1, 0x0002, ar17,
          0, 0, 0, 0, 0, 0)
  XTREG(19,  76, 32, 4, 4, 0x0112, 0x0006, -2, 1, 0x0002, ar18,
          0, 0, 0, 0, 0, 0)
  XTREG(20,  80, 32, 4, 4, 0x0113, 0x0006, -2, 1, 0x0002, ar19,
          0, 0, 0, 0, 0, 0)
  XTREG(21,  84, 32, 4, 4, 0x0114, 0x0006, -2, 1, 0x0002, ar20,
          0, 0, 0, 0, 0, 0)
  XTREG(22,  88, 32, 4, 4, 0x0115, 0x0006, -2, 1, 0x0002, ar21,
          0, 0, 0, 0, 0, 0)
  XTREG(23,  92, 32, 4, 4, 0x0116, 0x0006, -2, 1, 0x0002, ar22,
          0, 0, 0, 0, 0, 0)
  XTREG(24,  96, 32, 4, 4, 0x0117, 0x0006, -2, 1, 0x0002, ar23,
          0, 0, 0, 0, 0, 0)
  XTREG(25, 100, 32, 4, 4, 0x0118, 0x0006, -2, 1, 0x0002, ar24,
          0, 0, 0, 0, 0, 0)
  XTREG(26, 104, 32, 4, 4, 0x0119, 0x0006, -2, 1, 0x0002, ar25,
          0, 0, 0, 0, 0, 0)
  XTREG(27, 108, 32, 4, 4, 0x011a, 0x0006, -2, 1, 0x0002, ar26,
          0, 0, 0, 0, 0, 0)
  XTREG(28, 112, 32, 4, 4, 0x011b, 0x0006, -2, 1, 0x0002, ar27,
          0, 0, 0, 0, 0, 0)
  XTREG(29, 116, 32, 4, 4, 0x011c, 0x0006, -2, 1, 0x0002, ar28,
          0, 0, 0, 0, 0, 0)
  XTREG(30, 120, 32, 4, 4, 0x011d, 0x0006, -2, 1, 0x0002, ar29,
          0, 0, 0, 0, 0, 0)
  XTREG(31, 124, 32, 4, 4, 0x011e, 0x0006, -2, 1, 0x0002, ar30,
          0, 0, 0, 0, 0, 0)
  XTREG(32, 128, 32, 4, 4, 0x011f, 0x0006, -2, 1, 0x0002, ar31,
          0, 0, 0, 0, 0, 0)
  XTREG(33, 132, 32, 4, 4, 0x0120, 0x0006, -2, 1, 0x0002, ar32,
          0, 0, 0, 0, 0, 0)
  XTREG(34, 136, 32, 4, 4, 0x0121, 0x0006, -2, 1, 0x0002, ar33,
          0, 0, 0, 0, 0, 0)
  XTREG(35, 140, 32, 4, 4, 0x0122, 0x0006, -2, 1, 0x0002, ar34,
          0, 0, 0, 0, 0, 0)
  XTREG(36, 144, 32, 4, 4, 0x0123, 0x0006, -2, 1, 0x0002, ar35,
          0, 0, 0, 0, 0, 0)
  XTREG(37, 148, 32, 4, 4, 0x0124, 0x0006, -2, 1, 0x0002, ar36,
          0, 0, 0, 0, 0, 0)
  XTREG(38, 152, 32, 4, 4, 0x0125, 0x0006, -2, 1, 0x0002, ar37,
          0, 0, 0, 0, 0, 0)
  XTREG(39, 156, 32, 4, 4, 0x0126, 0x0006, -2, 1, 0x0002, ar38,
          0, 0, 0, 0, 0, 0)
  XTREG(40, 160, 32, 4, 4, 0x0127, 0x0006, -2, 1, 0x0002, ar39,
          0, 0, 0, 0, 0, 0)
  XTREG(41, 164, 32, 4, 4, 0x0128, 0x0006, -2, 1, 0x0002, ar40,
          0, 0, 0, 0, 0, 0)
  XTREG(42, 168, 32, 4, 4, 0x0129, 0x0006, -2, 1, 0x0002, ar41,
          0, 0, 0, 0, 0, 0)
  XTREG(43, 172, 32, 4, 4, 0x012a, 0x0006, -2, 1, 0x0002, ar42,
          0, 0, 0, 0, 0, 0)
  XTREG(44, 176, 32, 4, 4, 0x012b, 0x0006, -2, 1, 0x0002, ar43,
          0, 0, 0, 0, 0, 0)
  XTREG(45, 180, 32, 4, 4, 0x012c, 0x0006, -2, 1, 0x0002, ar44,
          0, 0, 0, 0, 0, 0)
  XTREG(46, 184, 32, 4, 4, 0x012d, 0x0006, -2, 1, 0x0002, ar45,
          0, 0, 0, 0, 0, 0)
  XTREG(47, 188, 32, 4, 4, 0x012e, 0x0006, -2, 1, 0x0002, ar46,
          0, 0, 0, 0, 0, 0)
  XTREG(48, 192, 32, 4, 4, 0x012f, 0x0006, -2, 1, 0x0002, ar47,
          0, 0, 0, 0, 0, 0)
  XTREG(49, 196, 32, 4, 4, 0x0130, 0x0006, -2, 1, 0x0002, ar48,
          0, 0, 0, 0, 0, 0)
  XTREG(50, 200, 32, 4, 4, 0x0131, 0x0006, -2, 1, 0x0002, ar49,
          0, 0, 0, 0, 0, 0)
  XTREG(51, 204, 32, 4, 4, 0x0132, 0x0006, -2, 1, 0x0002, ar50,
          0, 0, 0, 0, 0, 0)
  XTREG(52, 208, 32, 4, 4, 0x0133, 0x0006, -2, 1, 0x0002, ar51,
          0, 0, 0, 0, 0, 0)
  XTREG(53, 212, 32, 4, 4, 0x0134, 0x0006, -2, 1, 0x0002, ar52,
          0, 0, 0, 0, 0, 0)
  XTREG(54, 216, 32, 4, 4, 0x0135, 0x0006, -2, 1, 0x0002, ar53,
          0, 0, 0, 0, 0, 0)
  XTREG(55, 220, 32, 4, 4, 0x0136, 0x0006, -2, 1, 0x0002, ar54,
          0, 0, 0, 0, 0, 0)
  XTREG(56, 224, 32, 4, 4, 0x0137, 0x0006, -2, 1, 0x0002, ar55,
          0, 0, 0, 0, 0, 0)
  XTREG(57, 228, 32, 4, 4, 0x0138, 0x0006, -2, 1, 0x0002, ar56,
          0, 0, 0, 0, 0, 0)
  XTREG(58, 232, 32, 4, 4, 0x0139, 0x0006, -2, 1, 0x0002, ar57,
          0, 0, 0, 0, 0, 0)
  XTREG(59, 236, 32, 4, 4, 0x013a, 0x0006, -2, 1, 0x0002, ar58,
          0, 0, 0, 0, 0, 0)
  XTREG(60, 240, 32, 4, 4, 0x013b, 0x0006, -2, 1, 0x0002, ar59,
          0, 0, 0, 0, 0, 0)
  XTREG(61, 244, 32, 4, 4, 0x013c, 0x0006, -2, 1, 0x0002, ar60,
          0, 0, 0, 0, 0, 0)
  XTREG(62, 248, 32, 4, 4, 0x013d, 0x0006, -2, 1, 0x0002, ar61,
          0, 0, 0, 0, 0, 0)
  XTREG(63, 252, 32, 4, 4, 0x013e, 0x0006, -2, 1, 0x0002, ar62,
          0, 0, 0, 0, 0, 0)
  XTREG(64, 256, 32, 4, 4, 0x013f, 0x0006, -2, 1, 0x0002, ar63,
          0, 0, 0, 0, 0, 0)
  XTREG(65, 260, 32, 4, 4, 0x0200, 0x0006, -2, 2, 0x1100, lbeg,
          0, 0, 0, 0, 0, 0)
  XTREG(66, 264, 32, 4, 4, 0x0201, 0x0006, -2, 2, 0x1100, lend,
          0, 0, 0, 0, 0, 0)
  XTREG(67, 268, 32, 4, 4, 0x0202, 0x0006, -2, 2, 0x1100, lcount,
          0, 0, 0, 0, 0, 0)
  XTREG(68, 272,  6, 4, 4, 0x0203, 0x0006, -2, 2, 0x1100, sar,
          0, 0, 0, 0, 0, 0)
  XTREG(69, 276, 32, 4, 4, 0x0205, 0x0006, -2, 2, 0x1100, litbase,
          0, 0, 0, 0, 0, 0)
  XTREG(70, 280,  4, 4, 4, 0x0248, 0x0006, -2, 2, 0x1002, windowbase,
          0, 0, 0, 0, 0, 0)
  XTREG(71, 284, 16, 4, 4, 0x0249, 0x0006, -2, 2, 0x1002, windowstart,
          0, 0, 0, 0, 0, 0)
  XTREG(72, 288, 32, 4, 4, 0x02b0, 0x0002, -2, 2, 0x1000, sr176,
          0, 0, 0, 0, 0, 0)
  XTREG(73, 292, 32, 4, 4, 0x02d0, 0x0002, -2, 2, 0x1000, sr208,
          0, 0, 0, 0, 0, 0)
  XTREG(74, 296, 19, 4, 4, 0x02e6, 0x0006, -2, 2, 0x1100, ps,
          0, 0, 0, 0, 0, 0)
  XTREG(75, 300, 32, 4, 4, 0x03e7, 0x0006, -2, 3, 0x0110, threadptr,
          0, 0, 0, 0, 0, 0)
  XTREG(76, 304, 32, 4, 4, 0x020c, 0x0006, -1, 2, 0x1100, scompare1,
          0, 0, 0, 0, 0, 0)
  XTREG(77, 308, 32, 4, 4, 0x0327, 0x000e, -1, 3, 0x0210, expstate,
          0, 0, 0, 0, 0, 0)
  XTREG(78, 312, 32, 4, 4, 0x0300, 0x0006,  2, 3, 0x0210, stage1,
          0, 0, 0, 0, 0, 0)
  XTREG(79, 316, 32, 4, 4, 0x0301, 0x0006,  2, 3, 0x0210, stage2,
          0, 0, 0, 0, 0, 0)
  XTREG(80, 320, 32, 4, 4, 0x0302, 0x0006,  2, 3, 0x0210, input_align_reg,
          0, 0, 0, 0, 0, 0)
  XTREG(81, 324,  6, 4, 4, 0x0303, 0x0006,  2, 3, 0x0210, input_align_reg_pos,
          0, 0, 0, 0, 0, 0)
  XTREG(82, 328, 32, 4, 4, 0x0304, 0x0006,  2, 3, 0x0210, data_reg,
          0, 0, 0, 0, 0, 0)
  XTREG(83, 332,  7, 4, 4, 0x0305, 0x0006,  2, 3, 0x0210, data_reg_pos,
          0, 0, 0, 0, 0, 0)
  XTREG(84, 336, 32, 4, 4, 0x0306, 0x0006,  2, 3, 0x0210, crc_reg,
          0, 0, 0, 0, 0, 0)
  XTREG(85, 340, 32, 4, 4, 0x0307, 0x0006,  2, 3, 0x0210, pol_reg00,
          0, 0, 0, 0, 0, 0)
  XTREG(86, 344, 32, 4, 4, 0x0308, 0x0006,  2, 3, 0x0210, pol_reg01,
          0, 0, 0, 0, 0, 0)
  XTREG(87, 348, 32, 4, 4, 0x0309, 0x0006,  2, 3, 0x0210, pol_reg02,
          0, 0, 0, 0, 0, 0)
  XTREG(88, 352, 32, 4, 4, 0x030a, 0x0006,  2, 3, 0x0210, pol_reg03,
          0, 0, 0, 0, 0, 0)
  XTREG(89, 356, 32, 4, 4, 0x030b, 0x0006,  2, 3, 0x0210, pol_reg04,
          0, 0, 0, 0, 0, 0)
  XTREG(90, 360, 32, 4, 4, 0x030c, 0x0006,  2, 3, 0x0210, pol_reg05,
          0, 0, 0, 0, 0, 0)
  XTREG(91, 364, 32, 4, 4, 0x030d, 0x0006,  2, 3, 0x0210, pol_reg06,
          0, 0, 0, 0, 0, 0)
  XTREG(92, 368, 32, 4, 4, 0x030e, 0x0006,  2, 3, 0x0210, pol_reg07,
          0, 0, 0, 0, 0, 0)
  XTREG(93, 372, 32, 4, 4, 0x030f, 0x0006,  2, 3, 0x0210, pol_reg08,
          0, 0, 0, 0, 0, 0)
  XTREG(94, 376, 32, 4, 4, 0x0310, 0x0006,  2, 3, 0x0210, pol_reg09,
          0, 0, 0, 0, 0, 0)
  XTREG(95, 380, 32, 4, 4, 0x0311, 0x0006,  2, 3, 0x0210, pol_reg10,
          0, 0, 0, 0, 0, 0)
  XTREG(96, 384, 32, 4, 4, 0x0312, 0x0006,  2, 3, 0x0210, pol_reg11,
          0, 0, 0, 0, 0, 0)
  XTREG(97, 388, 32, 4, 4, 0x0313, 0x0006,  2, 3, 0x0210, pol_reg12,
          0, 0, 0, 0, 0, 0)
  XTREG(98, 392, 32, 4, 4, 0x0314, 0x0006,  2, 3, 0x0210, pol_reg13,
          0, 0, 0, 0, 0, 0)
  XTREG(99, 396, 32, 4, 4, 0x0315, 0x0006,  2, 3, 0x0210, pol_reg14,
          0, 0, 0, 0, 0, 0)
  XTREG(100, 400, 32, 4, 4, 0x0316, 0x0006,  2, 3, 0x0210, pol_reg15,
          0, 0, 0, 0, 0, 0)
  XTREG(101, 404, 32, 4, 4, 0x0317, 0x0006,  2, 3, 0x0210, pol_reg16,
          0, 0, 0, 0, 0, 0)
  XTREG(102, 408, 32, 4, 4, 0x0318, 0x0006,  2, 3, 0x0210, pol_reg17,
          0, 0, 0, 0, 0, 0)
  XTREG(103, 412, 32, 4, 4, 0x0319, 0x0006,  2, 3, 0x0210, pol_reg18,
          0, 0, 0, 0, 0, 0)
  XTREG(104, 416, 32, 4, 4, 0x031a, 0x0006,  2, 3, 0x0210, pol_reg19,
          0, 0, 0, 0, 0, 0)
  XTREG(105, 420, 32, 4, 4, 0x031b, 0x0006,  2, 3, 0x0210, pol_reg20,
          0, 0, 0, 0, 0, 0)
  XTREG(106, 424, 32, 4, 4, 0x031c, 0x0006,  2, 3, 0x0210, pol_reg21,
          0, 0, 0, 0, 0, 0)
  XTREG(107, 428, 32, 4, 4, 0x031d, 0x0006,  2, 3, 0x0210, pol_reg22,
          0, 0, 0, 0, 0, 0)
  XTREG(108, 432, 32, 4, 4, 0x031e, 0x0006,  2, 3, 0x0210, pol_reg23,
          0, 0, 0, 0, 0, 0)
  XTREG(109, 436, 32, 4, 4, 0x031f, 0x0006,  2, 3, 0x0210, pol_reg24,
          0, 0, 0, 0, 0, 0)
  XTREG(110, 440, 32, 4, 4, 0x0320, 0x0006,  2, 3, 0x0210, pol_reg25,
          0, 0, 0, 0, 0, 0)
  XTREG(111, 444, 32, 4, 4, 0x0321, 0x0006,  2, 3, 0x0210, pol_reg26,
          0, 0, 0, 0, 0, 0)
  XTREG(112, 448, 32, 4, 4, 0x0322, 0x0006,  2, 3, 0x0210, pol_reg27,
          0, 0, 0, 0, 0, 0)
  XTREG(113, 452, 32, 4, 4, 0x0323, 0x0006,  2, 3, 0x0210, pol_reg28,
          0, 0, 0, 0, 0, 0)
  XTREG(114, 456, 32, 4, 4, 0x0324, 0x0006,  2, 3, 0x0210, pol_reg29,
          0, 0, 0, 0, 0, 0)
  XTREG(115, 460, 32, 4, 4, 0x0325, 0x0006,  2, 3, 0x0210, pol_reg30,
          0, 0, 0, 0, 0, 0)
  XTREG(116, 464, 32, 4, 4, 0x0326, 0x0006,  2, 3, 0x0210, pol_reg31,
          0, 0, 0, 0, 0, 0)
  XTREG(117, 468, 32, 4, 4, 0x0259, 0x000d, -2, 2, 0x1000, mmid,
          0, 0, 0, 0, 0, 0)
  XTREG(118, 472,  2, 4, 4, 0x0260, 0x0007, -2, 2, 0x1000, ibreakenable,
          0, 0, 0, 0, 0, 0)
  XTREG(119, 476,  6, 4, 4, 0x0263, 0x0007, -2, 2, 0x1000, atomctl,
          0, 0, 0, 0, 0, 0)
  XTREG(120, 480, 32, 4, 4, 0x0268, 0x0007, -2, 2, 0x1000, ddr,
          0, 0, 0, 0, 0, 0)
  XTREG(121, 484, 32, 4, 4, 0x0280, 0x0007, -2, 2, 0x1000, ibreaka0,
          0, 0, 0, 0, 0, 0)
  XTREG(122, 488, 32, 4, 4, 0x0281, 0x0007, -2, 2, 0x1000, ibreaka1,
          0, 0, 0, 0, 0, 0)
  XTREG(123, 492, 32, 4, 4, 0x0290, 0x0007, -2, 2, 0x1000, dbreaka0,
          0, 0, 0, 0, 0, 0)
  XTREG(124, 496, 32, 4, 4, 0x0291, 0x0007, -2, 2, 0x1000, dbreaka1,
          0, 0, 0, 0, 0, 0)
  XTREG(125, 500, 32, 4, 4, 0x02a0, 0x0007, -2, 2, 0x1000, dbreakc0,
          0, 0, 0, 0, 0, 0)
  XTREG(126, 504, 32, 4, 4, 0x02a1, 0x0007, -2, 2, 0x1000, dbreakc1,
          0, 0, 0, 0, 0, 0)
  XTREG(127, 508, 32, 4, 4, 0x02b1, 0x0007, -2, 2, 0x1000, epc1,
          0, 0, 0, 0, 0, 0)
  XTREG(128, 512, 32, 4, 4, 0x02b2, 0x0007, -2, 2, 0x1000, epc2,
          0, 0, 0, 0, 0, 0)
  XTREG(129, 516, 32, 4, 4, 0x02b3, 0x0007, -2, 2, 0x1000, epc3,
          0, 0, 0, 0, 0, 0)
  XTREG(130, 520, 32, 4, 4, 0x02b4, 0x0007, -2, 2, 0x1000, epc4,
          0, 0, 0, 0, 0, 0)
  XTREG(131, 524, 32, 4, 4, 0x02b5, 0x0007, -2, 2, 0x1000, epc5,
          0, 0, 0, 0, 0, 0)
  XTREG(132, 528, 32, 4, 4, 0x02b6, 0x0007, -2, 2, 0x1000, epc6,
          0, 0, 0, 0, 0, 0)
  XTREG(133, 532, 32, 4, 4, 0x02c0, 0x0007, -2, 2, 0x1000, depc,
          0, 0, 0, 0, 0, 0)
  XTREG(134, 536, 19, 4, 4, 0x02c2, 0x0007, -2, 2, 0x1000, eps2,
          0, 0, 0, 0, 0, 0)
  XTREG(135, 540, 19, 4, 4, 0x02c3, 0x0007, -2, 2, 0x1000, eps3,
          0, 0, 0, 0, 0, 0)
  XTREG(136, 544, 19, 4, 4, 0x02c4, 0x0007, -2, 2, 0x1000, eps4,
          0, 0, 0, 0, 0, 0)
  XTREG(137, 548, 19, 4, 4, 0x02c5, 0x0007, -2, 2, 0x1000, eps5,
          0, 0, 0, 0, 0, 0)
  XTREG(138, 552, 19, 4, 4, 0x02c6, 0x0007, -2, 2, 0x1000, eps6,
          0, 0, 0, 0, 0, 0)
  XTREG(139, 556, 32, 4, 4, 0x02d1, 0x0007, -2, 2, 0x1000, excsave1,
          0, 0, 0, 0, 0, 0)
  XTREG(140, 560, 32, 4, 4, 0x02d2, 0x0007, -2, 2, 0x1000, excsave2,
          0, 0, 0, 0, 0, 0)
  XTREG(141, 564, 32, 4, 4, 0x02d3, 0x0007, -2, 2, 0x1000, excsave3,
          0, 0, 0, 0, 0, 0)
  XTREG(142, 568, 32, 4, 4, 0x02d4, 0x0007, -2, 2, 0x1000, excsave4,
          0, 0, 0, 0, 0, 0)
  XTREG(143, 572, 32, 4, 4, 0x02d5, 0x0007, -2, 2, 0x1000, excsave5,
          0, 0, 0, 0, 0, 0)
  XTREG(144, 576, 32, 4, 4, 0x02d6, 0x0007, -2, 2, 0x1000, excsave6,
          0, 0, 0, 0, 0, 0)
  XTREG(145, 580,  4, 4, 4, 0x02e0, 0x0007, -2, 2, 0x1000, cpenable,
          0, 0, 0, 0, 0, 0)
  XTREG(146, 584, 13, 4, 4, 0x02e2, 0x000b, -2, 2, 0x1000, interrupt,
          0, 0, 0, 0, 0, 0)
  XTREG(147, 588, 13, 4, 4, 0x02e2, 0x000d, -2, 2, 0x1000, intset,
          0, 0, 0, 0, 0, 0)
  XTREG(148, 592, 13, 4, 4, 0x02e3, 0x000d, -2, 2, 0x1000, intclear,
          0, 0, 0, 0, 0, 0)
  XTREG(149, 596, 13, 4, 4, 0x02e4, 0x0007, -2, 2, 0x1000, intenable,
          0, 0, 0, 0, 0, 0)
  XTREG(150, 600, 32, 4, 4, 0x02e7, 0x0007, -2, 2, 0x1000, vecbase,
          0, 0, 0, 0, 0, 0)
  XTREG(151, 604,  6, 4, 4, 0x02e8, 0x0007, -2, 2, 0x1000, exccause,
          0, 0, 0, 0, 0, 0)
  XTREG(152, 608, 12, 4, 4, 0x02e9, 0x0003, -2, 2, 0x1000, debugcause,
          0, 0, 0, 0, 0, 0)
  XTREG(153, 612, 32, 4, 4, 0x02ea, 0x000f, -2, 2, 0x1000, ccount,
          0, 0, 0, 0, 0, 0)
  XTREG(154, 616, 32, 4, 4, 0x02eb, 0x0003, -2, 2, 0x1000, prid,
          0, 0, 0, 0, 0, 0)
  XTREG(155, 620, 32, 4, 4, 0x02ec, 0x000f, -2, 2, 0x1000, icount,
          0, 0, 0, 0, 0, 0)
  XTREG(156, 624,  4, 4, 4, 0x02ed, 0x0007, -2, 2, 0x1000, icountlevel,
          0, 0, 0, 0, 0, 0)
  XTREG(157, 628, 32, 4, 4, 0x02ee, 0x0007, -2, 2, 0x1000, excvaddr,
          0, 0, 0, 0, 0, 0)
  XTREG(158, 632, 32, 4, 4, 0x02f0, 0x000f, -2, 2, 0x1000, ccompare0,
          0, 0, 0, 0, 0, 0)
  XTREG(159, 636, 32, 4, 4, 0x02f1, 0x000f, -2, 2, 0x1000, ccompare1,
          0, 0, 0, 0, 0, 0)
  XTREG(160, 640, 32, 4, 4, 0x0000, 0x0006, -2, 8, 0x0100, a0,
          0, 0, 0, 0, 0, 0)
  XTREG(161, 644, 32, 4, 4, 0x0001, 0x0006, -2, 8, 0x0100, a1,
          0, 0, 0, 0, 0, 0)
  XTREG(162, 648, 32, 4, 4, 0x0002, 0x0006, -2, 8, 0x0100, a2,
          0, 0, 0, 0, 0, 0)
  XTREG(163, 652, 32, 4, 4, 0x0003, 0x0006, -2, 8, 0x0100, a3,
          0, 0, 0, 0, 0, 0)
  XTREG(164, 656, 32, 4, 4, 0x0004, 0x0006, -2, 8, 0x0100, a4,
          0, 0, 0, 0, 0, 0)
  XTREG(165, 660, 32, 4, 4, 0x0005, 0x0006, -2, 8, 0x0100, a5,
          0, 0, 0, 0, 0, 0)
  XTREG(166, 664, 32, 4, 4, 0x0006, 0x0006, -2, 8, 0x0100, a6,
          0, 0, 0, 0, 0, 0)
  XTREG(167, 668, 32, 4, 4, 0x0007, 0x0006, -2, 8, 0x0100, a7,
          0, 0, 0, 0, 0, 0)
  XTREG(168, 672, 32, 4, 4, 0x0008, 0x0006, -2, 8, 0x0100, a8,
          0, 0, 0, 0, 0, 0)
  XTREG(169, 676, 32, 4, 4, 0x0009, 0x0006, -2, 8, 0x0100, a9,
          0, 0, 0, 0, 0, 0)
  XTREG(170, 680, 32, 4, 4, 0x000a, 0x0006, -2, 8, 0x0100, a10,
          0, 0, 0, 0, 0, 0)
  XTREG(171, 684, 32, 4, 4, 0x000b, 0x0006, -2, 8, 0x0100, a11,
          0, 0, 0, 0, 0, 0)
  XTREG(172, 688, 32, 4, 4, 0x000c, 0x0006, -2, 8, 0x0100, a12,
          0, 0, 0, 0, 0, 0)
  XTREG(173, 692, 32, 4, 4, 0x000d, 0x0006, -2, 8, 0x0100, a13,
          0, 0, 0, 0, 0, 0)
  XTREG(174, 696, 32, 4, 4, 0x000e, 0x0006, -2, 8, 0x0100, a14,
          0, 0, 0, 0, 0, 0)
  XTREG(175, 700, 32, 4, 4, 0x000f, 0x0006, -2, 8, 0x0100, a15,
          0, 0, 0, 0, 0, 0)
